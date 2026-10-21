#!/usr/bin/env python3
"""Atomically publish an external assisted-run ACK or stop request.

This noncritical publisher does not import Isaac or change worker/controller
sources. All external control writers must use this entry point. The exclusive
lock serializes cooperating publishers; it cannot protect against a writer
that bypasses this tool and directly truncates the destination.
"""
from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import sys
import tempfile
import threading
import time
import tarfile
from unittest import mock


NAMES = {'ack': 'startup_image_review.json', 'stop': 'stop_request.json'}
CAMERAS = {'observation.images.isaac_3d', 'observation.images.left_wrist',
           'observation.images.right_wrist'}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(value, message):
    if not value:
        raise ValueError(message)


def text_field(message, key):
    require(isinstance(message.get(key), str) and bool(message[key].strip()), 'missing '+key)


def valid_stamp(value):
    return (isinstance(value, dict) and value.get('valid') is True
            and all(type(value.get(k)) is int and value[k] >= 0 for k in ('epoch', 'index'))
            and all(type(value.get(k)) in (int, float) and math.isfinite(value[k]) for k in ('time', 'dt'))
            and value['time'] >= 0 and value['dt'] > 0)


def validate_message(run_dir, run_name, kind, message):
    require(kind in NAMES, 'unknown control kind')
    require(run_dir.is_dir() and run_dir.name == run_name, 'run directory identity mismatch')
    require((run_dir/'started_at').is_file() and (run_dir/'source_before.json').is_file(), 'run has not started')
    require(isinstance(message, dict) and message.get('run_name') == run_name, 'message run identity mismatch')
    if 'run_dir' in message:
        require(Path(message['run_dir']).resolve() == run_dir, 'message run directory mismatch')
    # Serialize before creating any visible control path, rejecting NaN/Inf and
    # unsupported values even in optional fields.
    encoded = (json.dumps(message, ensure_ascii=False, sort_keys=True, allow_nan=False)+'\n').encode()
    if kind == 'stop':
        text_field(message, 'reason')
        if 'physics_stamp' in message:
            require(valid_stamp(message['physics_stamp']), 'invalid stop physics stamp')
        return encoded
    require(not (run_dir/'finished_at').exists(), 'run already finished')
    require(message.get('accepted') is True, 'ACK must explicitly accept the reviewed images')
    text_field(message, 'reviewer')
    text_field(message, 'scope')
    require(not (run_dir/NAMES['stop']).exists(), 'run already has a stop request')
    ready = json.loads((run_dir/'startup_images_ready.json').read_text())
    require(valid_stamp(ready['physics_stamp']) and message.get('physics_stamp') == ready['physics_stamp'], 'ACK physics stamp mismatch')
    require(ready['before_render_physics_stamp'] == ready['physics_stamp'], 'startup rendering changed physics stamp')
    rows = ready['images']
    require(isinstance(rows, list) and len(rows) == 3 and {v['camera'] for v in rows} == CAMERAS, 'three original images required')
    expected = {}
    for row in rows:
        path = Path(row['path']).resolve()
        require(path.parent == run_dir and path.name == 'startup_'+row['camera'].replace('.', '_')+'.png', 'image belongs to another run')
        require(path.is_file() and digest(path) == row['sha256'], 'startup image bytes changed')
        expected[row['camera']] = row['sha256']
    require(message.get('image_sha256') == expected, 'ACK image SHA map mismatch')
    return encoded


def publish(run_dir, run_name, kind, message):
    """Expose the final path once, with complete durable JSON already present."""
    run_dir = Path(run_dir).resolve()
    require(kind in NAMES, 'unknown control kind')
    require(run_dir.is_dir() and run_dir.name == run_name, 'run directory identity mismatch')
    destination = run_dir/NAMES[kind]
    lock = run_dir/('.'+NAMES[kind]+'.publish.lock')
    # O_EXCL rejects a simultaneous publisher instead of queueing a stale ACK.
    lock_fd = os.open(lock, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    temporary = None
    try:
        if os.path.lexists(destination):
            raise FileExistsError('control file already exists: '+str(destination))
        encoded = validate_message(run_dir, run_name, kind, message)
        with tempfile.NamedTemporaryFile(mode='wb', dir=run_dir,
                prefix='.'+NAMES[kind]+'.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        if os.path.lexists(destination):
            raise FileExistsError('control file appeared during publication: '+str(destination))
        os.replace(temporary, destination)
        temporary = None
        directory_fd = os.open(run_dir, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return dict(published=True, kind=kind, run_name=run_name,
                    path=str(destination), sha256=digest(destination), bytes=len(encoded))
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
        os.close(lock_fd)
        lock.unlink(missing_ok=True)


def publish_stop_request(run_dir, detail):
    """Guard entry point: preserve an existing stop and wait out its publisher."""
    run_dir=Path(run_dir).resolve()
    message=dict(detail,run_name=run_dir.name)
    deadline=time.monotonic()+2.
    while True:
        try:
            return publish(run_dir,run_dir.name,'stop',message)
        except FileExistsError:
            if os.path.lexists(run_dir/NAMES['stop']):
                return dict(published=False,reason='existing complete stop preserved')
            if time.monotonic()>=deadline:
                raise
            time.sleep(.005)


def worker_ack_reader(worker_path):
    """Extract the worker's unchanged parse and image acceptance statements."""
    tree = ast.parse(worker_path.read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'review_initial_images')
    index = next(i for i,n in enumerate(fn.body) if isinstance(n, ast.Assign)
                 and any(isinstance(v, ast.Name) and v.id == 'approved' for v in n.targets))
    statements = fn.body[index:index+2]
    require(isinstance(statements[1], ast.If), 'worker ACK protocol changed')
    code = compile(ast.fix_missing_locations(ast.Module(body=statements, type_ignores=[])), str(worker_path), 'exec')
    def read(path, rows):
        namespace = dict(json=json, ack=path, rows=rows)
        exec(code, namespace)
        return namespace['approved']
    return read, dict(source_sha256=digest(worker_path), lines=[n.lineno for n in statements],
                     ast_sha256=hashlib.sha256(ast.dump(ast.Module(body=statements, type_ignores=[]), include_attributes=False).encode()).hexdigest())


def watcher_change_proof():
    path=Path(__file__).with_name('watch_visible_meshes.py')
    archive=Path(__file__).resolve().parents[1]/'versions/assisted_v18_tools_before_A14_beam_full.tar.gz'
    with tarfile.open(archive) as stream:
        old=stream.extractfile('watch_visible_meshes.py').read()
    original=hashlib.sha256(old).hexdigest()
    require(original=='23cf3acd74182d5ae27f35c0b47bbb21cca53e644ada57a1274983da5d61238f','wrong watcher baseline')
    expected=ast.parse(old);count=0
    class WritesOnly(ast.NodeTransformer):
        def visit_Expr(self,node):
            nonlocal count
            call=node.value
            if (isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute)
                    and call.func.attr=='write_text' and isinstance(call.func.value,ast.BinOp)
                    and isinstance(call.func.value.right,ast.Constant) and call.func.value.right.value=='stop_request.json'):
                payload=call.args[0].left.args[0]
                count+=1
                return ast.Expr(value=ast.Call(func=ast.Name(id='publish_stop_request',ctx=ast.Load()),args=[ast.Name(id='D',ctx=ast.Load()),payload],keywords=[]))
            return self.generic_visit(node)
    expected=WritesOnly().visit(expected)
    at=next(i for i,n in enumerate(expected.body) if isinstance(n,ast.ImportFrom) and n.module=='visual_support_check')+1
    expected.body.insert(at,ast.ImportFrom(module='publish_runtime_control',names=[ast.alias(name='publish_stop_request')],level=0))
    matched=count==2 and ast.dump(expected,include_attributes=False)==ast.dump(ast.parse(path.read_text()),include_attributes=False)
    require(matched,'watcher changed beyond import and two atomic stop publications')
    return dict(watcher_sha256=digest(path),watcher_original_sha256=original,watcher_atomic_stop_only_ast=matched,watcher_original_archive_sha256=digest(archive))


def self_test(output):
    """Isolated filesystem/concurrency fixtures; no native process is created."""
    reader, binding = worker_ack_reader(Path(__file__).with_name('assisted_worker.py'))
    watcher_proof=watcher_change_proof()
    worker=Path(__file__).with_name('assisted_worker.py')
    act=next(n for n in ast.walk(ast.parse(worker.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='act')
    stop_assignment=next(n for n in ast.walk(act) if isinstance(n,ast.Assign) and any(isinstance(v,ast.Name) and v.id=='detail' for v in n.targets))
    stop_code=compile(ast.fix_missing_locations(ast.Module(body=[stop_assignment],type_ignores=[])),str(worker),'exec')
    def stop_reader(path):
        namespace=dict(json=json,request=path);exec(stop_code,namespace);return namespace['detail']
    checks = []
    with tempfile.TemporaryDirectory(prefix='beam_control_publish_') as scratch:
        def fixture():
            directory = Path(tempfile.mkdtemp(prefix='A15_fixture_', dir=scratch))
            (directory/'started_at').write_text('fixture\n')
            (directory/'source_before.json').write_text('{}\n')
            stamp = dict(valid=True, epoch=0, index=3, time=3/240, dt=1/240,
                         identity_source='isolated_fixture')
            rows = []
            for camera in sorted(CAMERAS):
                image = directory/('startup_'+camera.replace('.', '_')+'.png')
                image.write_bytes(('isolated image bytes '+camera).encode())
                rows.append(dict(camera=camera, path=str(image), sha256=digest(image)))
            (directory/'startup_images_ready.json').write_text(json.dumps(dict(
                physics_stamp=stamp, before_render_physics_stamp=stamp, images=rows)))
            message = dict(run_name=directory.name, accepted=True,
                reviewer='isolated fixture reviewer', scope='atomic-publication test only',
                image_sha256={v['camera']:v['sha256'] for v in rows}, physics_stamp=stamp)
            return directory, rows, message

        # Deterministic reproduction of A14: reader sees an existing empty file
        # while the legacy writer has opened/truncated but not written it.
        failures = 0
        for _ in range(5):
            directory, rows, message = fixture()
            path = directory/NAMES['ack']
            opened, continue_write = threading.Event(), threading.Event()
            def old_writer():
                with path.open('w') as stream:
                    opened.set()
                    require(continue_write.wait(5), 'legacy fixture timeout')
                    stream.write(json.dumps(message))
            thread = threading.Thread(target=old_writer)
            thread.start()
            require(opened.wait(5) and path.exists(), 'legacy empty path not observed')
            try:
                reader(path, rows)
            except json.JSONDecodeError:
                failures += 1
            finally:
                continue_write.set()
                thread.join(5)
        checks.append(dict(name='legacy_worker_empty_read_reproduced_5_of_5', passed=failures == 5))

        directory, rows, message = fixture()
        prepared, release, observed = threading.Event(), threading.Event(), threading.Event()
        actual_replace, actual_fsync = os.replace, os.fsync
        order, errors = [], []
        def paused_replace(source, destination):
            require(not Path(destination).exists(), 'control path visible before replace')
            require(json.loads(Path(source).read_text()) == message, 'temporary JSON incomplete')
            order.append('replace')
            prepared.set()
            require(release.wait(5), 'atomic fixture timeout')
            return actual_replace(source, destination)
        def fsync(fd):
            order.append('directory_fsync' if stat.S_ISDIR(os.fstat(fd).st_mode) else 'temporary_fsync')
            return actual_fsync(fd)
        def new_writer():
            try:
                publish(directory, directory.name, 'ack', message)
            except Exception as exc:
                errors.append(repr(exc))
        def new_reader():
            path = directory/NAMES['ack']
            while not path.exists():
                if observed.wait(.001):
                    return
            try:
                require(reader(path, rows) == message, 'worker read different complete message')
            except Exception as exc:
                errors.append(repr(exc))
            finally:
                observed.set()
        with mock.patch.object(os, 'replace', paused_replace), mock.patch.object(os, 'fsync', fsync):
            writer = threading.Thread(target=new_writer)
            consumer = threading.Thread(target=new_reader)
            writer.start(); consumer.start()
            require(prepared.wait(5), 'complete temporary was not prepared')
            invisible = not (directory/NAMES['ack']).exists()
            release.set(); writer.join(5); consumer.join(5)
        checks.append(dict(name='new_path_invisible_until_complete_bytes', passed=invisible and observed.is_set() and not errors, errors=errors))
        checks.append(dict(name='temporary_fsync_replace_directory_fsync_order', passed=order == ['temporary_fsync','replace','directory_fsync'], observed=order))

        mutations = [('wrong_run', lambda d,r,m:m.update(run_name='other_run')),
            ('wrong_image_map', lambda d,r,m:m['image_sha256'].pop(next(iter(CAMERAS)))),
            ('wrong_image_hash', lambda d,r,m:m['image_sha256'].update({next(iter(CAMERAS)):'0'*64})),
            ('changed_image_bytes', lambda d,r,m:Path(r[0]['path']).write_bytes(b'changed')),
            ('missing_accepted', lambda d,r,m:m.pop('accepted')),
            ('rejected_accepted', lambda d,r,m:m.update(accepted=False)),
            ('missing_reviewer', lambda d,r,m:m.pop('reviewer')),
            ('missing_scope', lambda d,r,m:m.pop('scope')),
            ('missing_stamp', lambda d,r,m:m.pop('physics_stamp')),
            ('wrong_stamp', lambda d,r,m:m.update(physics_stamp={**m['physics_stamp'],'index':4})),
            ('nonfinite_optional_value', lambda d,r,m:m.update(extra=float('nan')))]
        for name,change in mutations:
            directory, rows, message = fixture();change(directory, rows, message)
            rejected = False
            try:
                publish(directory, directory.name, 'ack', message)
            except (ValueError, KeyError, FileExistsError):
                rejected = True
            checks.append(dict(name=name+'_rejected_before_visible_path', passed=rejected and not (directory/NAMES['ack']).exists()))

        directory, rows, message = fixture()
        path = directory/NAMES['ack'];path.write_bytes(b'existing control stays unchanged')
        try:
            publish(directory, directory.name, 'ack', message)
            rejected = False
        except FileExistsError:
            rejected = path.read_bytes() == b'existing control stays unchanged'
        checks.append(dict(name='existing_control_never_overwritten', passed=rejected))
        directory, rows, message = fixture();barrier=threading.Barrier(8);winners=[];rejects=[]
        def duplicate():
            barrier.wait()
            try:
                winners.append(publish(directory, directory.name, 'ack', message))
            except (ValueError, FileExistsError):
                rejects.append(True)
        threads=[threading.Thread(target=duplicate) for _ in range(8)]
        for thread in threads:thread.start()
        for thread in threads:thread.join(5)
        checks.append(dict(name='eight_concurrent_publishers_exactly_one_message', passed=len(winners)==1 and len(rejects)==7 and reader(directory/NAMES['ack'],rows)==message, published=len(winners), rejected=len(rejects)))
        directory, rows, message = fixture()
        stop=dict(run_name=directory.name,reason='isolated stop protocol fixture')
        publish(directory,directory.name,'stop',stop)
        checks.append(dict(name='atomic_stop_complete_json',passed=json.loads((directory/NAMES['stop']).read_text())==stop))
        try:
            publish(directory,directory.name,'ack',message);rejected=False
        except ValueError:rejected=True
        checks.append(dict(name='ack_after_stop_rejected',passed=rejected and not (directory/NAMES['ack']).exists()))
        stop_empty_reads=0
        for _ in range(5):
            directory,rows,message=fixture();path=directory/NAMES['stop'];opened=threading.Event();release=threading.Event()
            def legacy_stop():
                with path.open('w') as stream:
                    opened.set();require(release.wait(5),'legacy stop timeout');stream.write(json.dumps(dict(reason='fixture')))
            thread=threading.Thread(target=legacy_stop);thread.start();require(opened.wait(5),'legacy stop not opened')
            try:stop_reader(path)
            except json.JSONDecodeError:stop_empty_reads+=1
            finally:release.set();thread.join(5)
        checks.append(dict(name='legacy_worker_stop_empty_read_reproduced_5_of_5',passed=stop_empty_reads==5))
        directory,rows,message=fixture();stop=dict(reason='atomic guard fixture');observed_stop=[];stop_done=threading.Event()
        def observe_stop():
            path=directory/NAMES['stop']
            while not path.exists():
                if stop_done.wait(.001):return
            observed_stop.append(stop_reader(path))
        thread=threading.Thread(target=observe_stop);thread.start();publish_stop_request(directory,stop);thread.join(5);stop_done.set()
        checks.append(dict(name='worker_stop_reader_observes_complete_json',passed=observed_stop==[dict(stop,run_name=directory.name)]))
        directory,rows,message=fixture();barrier=threading.Barrier(8);winners=[];rejects=[]
        def duplicate_stop():
            barrier.wait()
            try:winners.append(publish(directory,directory.name,'stop',dict(stop,run_name=directory.name)))
            except (ValueError,FileExistsError):rejects.append(True)
        threads=[threading.Thread(target=duplicate_stop) for _ in range(8)]
        for thread in threads:thread.start()
        for thread in threads:thread.join(5)
        checks.append(dict(name='eight_concurrent_stop_publishers_exactly_one_message',passed=len(winners)==1 and len(rejects)==7 and stop_reader(directory/NAMES['stop'])==dict(stop,run_name=directory.name),published=len(winners),rejected=len(rejects)))
        existing=(directory/NAMES['stop']).read_bytes();result=publish_stop_request(directory,dict(reason='later duplicate guard'))
        checks.append(dict(name='guard_duplicate_preserves_first_stop',passed=result['published'] is False and (directory/NAMES['stop']).read_bytes()==existing))
        for name,bad in [('wrong_stop_run',dict(run_name='wrong',reason='fixture')),('missing_stop_reason',dict(run_name='CURRENT'))]:
            directory,rows,message=fixture()
            if bad['run_name']=='CURRENT':bad['run_name']=directory.name
            try:publish(directory,directory.name,'stop',bad);rejected=False
            except ValueError:rejected=True
            checks.append(dict(name=name+'_rejected',passed=rejected and not (directory/NAMES['stop']).exists()))
    result=dict(schema='beam_C03_control_publish_validation_v1',passed=all(v['passed'] for v in checks),
        tool_sha256=digest(__file__),worker_ack_reader=binding,checks=checks,
        worker_stop_reader=dict(source_sha256=digest(worker),line=stop_assignment.lineno,
            ast_sha256=hashlib.sha256(ast.dump(stop_assignment,include_attributes=False).encode()).hexdigest()),
        native_started=False,production_source_modified=False,worker_source_modified=False,
        critical_tools_modified=True,previous_critical_tool_count=18,current_critical_tool_count=19,
        **watcher_proof,old_direct_write_empty_read_reproduced=failures==stop_empty_reads==5,
        atomic_reader_observed_only_complete_json=not errors and observed.is_set() and bool(observed_stop),
        startup_image_binding_checked=all(v['passed'] for v in checks if any(k in v['name'] for k in ('image','stamp','accepted','reviewer','scope'))),
        duplicate_publish_rejected=all(v['passed'] for v in checks if any(k in v['name'] for k in ('concurrent','duplicate','overwritten'))),
        native_physics_executed=0,watcher_stop_writer_count=2,
        scope='Confirmed ACK/STOP JSON race; filesystem fixtures and unchanged worker reader ASTs; watcher AST differs only by import and two publication calls.',
        limits=['Cooperating external writers must use this publisher; direct writes bypass its lock.',
                'Worker/controller source is unchanged; only the two guard STOP publications use the same atomic interface.',
                'No native assembly, camera rendering, or control-motion validation is claimed.'])
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    require(result['passed'],'atomic control publication validation failed')
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-dir',type=Path)
    ap.add_argument('--run-name')
    ap.add_argument('--kind',choices=sorted(NAMES))
    ap.add_argument('--message-file',help='complete JSON file, or - for stdin')
    ap.add_argument('--self-test',action='store_true')
    ap.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'C03_control_publish_validation.json')
    args=ap.parse_args()
    if args.self_test:
        result=self_test(args.output)
        print(json.dumps(dict(passed=result['passed'],checks=len(result['checks']),tool_sha256=result['tool_sha256'],report=str(args.output))))
        return
    require(args.run_dir is not None and args.run_name and args.kind and args.message_file,'run-dir/run-name/kind/message-file required')
    message=json.loads(sys.stdin.read() if args.message_file=='-' else Path(args.message_file).read_text())
    print(json.dumps(publish(args.run_dir,args.run_name,args.kind,message)))


if __name__=='__main__':
    try:
        main()
    except (ValueError,KeyError,TypeError,OSError) as exc:
        print(json.dumps(dict(published=False,error=type(exc).__name__+': '+str(exc))))
        raise SystemExit(2)
