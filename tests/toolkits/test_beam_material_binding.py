"""Exercise real material binding logic without launching Isaac Sim."""
import ast
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
source=ast.parse((ROOT/'internutopia_extension/robots/ur5e.py').read_text())
method=next(n for n in ast.walk(source) if isinstance(n,ast.FunctionDef) and n.name=='_bind_explicit_gripper_material')
ns={};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'<material binding>','exec'),ns)
bind=ns['_bind_explicit_gripper_material']

class Prim:
    def __init__(self,path,collision=False,parent=None,proxy=False):
        self.path=path;self.collision=collision;self.parent=parent;self.proxy=proxy;self.children=[]
        self.bindings={'allPurpose':'visual'};self.enabled=True
        if parent:parent.children.append(self)
    def GetPath(self):return self.path
    def GetPrim(self):return self
    def IsValid(self):return True
    def HasAPI(self,api):return self.collision
    def IsInstanceProxy(self):return self.proxy
    def GetParent(self):return self.parent
    def GetAttribute(self,name):return NS(Get=lambda:self.enabled if name=='physics:collisionEnabled' else None)
    def __bool__(self):return True

def walk(p,*a):
    yield p
    for child in p.children:yield from walk(child)

class Binding:
    fail_resolution=False
    def __init__(self,p):self.p=p
    @staticmethod
    def Apply(p):
        if p.proxy:raise RuntimeError('Cannot author on instance proxy')
        return Binding(p)
    def ComputeBoundMaterial(self,materialPurpose):
        p=self.p
        while p:
            if materialPurpose in p.bindings:
                return (None if self.fail_resolution else p.bindings[materialPurpose]),None
            p=p.parent
        return None,None
    def Bind(self,material,bindingStrength,materialPurpose):
        assert materialPurpose=='physics';assert bindingStrength=='stronger'
        self.p.bindings[materialPurpose]=material

class MaterialBindingRegression(unittest.TestCase):
    def setUp(self):
        self.root=Prim('/robot');self.mat=Prim('/material')
        self.left=Prim('/robot/gripper/left',parent=self.root)
        self.right=Prim('/robot/gripper/right',parent=self.root)
        self.lc=Prim(self.left.path+'/shape',True,self.left)
        self.rc=Prim(self.right.path+'/shape',True,self.right)
        self.robot=NS(config=NS(prim_path='/robot',left_finger_link_name='left',right_finger_link_name='right'),
                      _rigid_body_map={},gripper_contact_material_diagnostics={'bound_finger_paths':[]})
        self.stage=NS(GetPrimAtPath=lambda p:self.root)
        pxr=NS(Usd=NS(PrimRange=walk,TraverseInstanceProxies=lambda:None),
               UsdPhysics=NS(CollisionAPI=object()),
               UsdShade=NS(Material=lambda p:p,MaterialBindingAPI=Binding,Tokens=NS(strongerThanDescendants='stronger')))
        self.patch=patch.dict(sys.modules,{'pxr':pxr});self.patch.start();self.addCleanup(self.patch.stop)
        Binding.fail_resolution=False
    def test_empty_body_cache_still_binds_real_colliders_and_keeps_visual_material(self):
        bind(self.robot,self.stage,self.mat)
        self.assertEqual(self.robot.gripper_contact_material_diagnostics['bound_finger_paths'],[self.left.path,self.right.path])
        for p in [self.lc,self.rc]:
            self.assertIs(p.bindings['physics'],self.mat)
            self.assertEqual(p.bindings['allPurpose'],'visual')
    def test_proxy_shape_is_bound_at_its_instance_ancestor(self):
        self.lc.proxy=True
        bind(self.robot,self.stage,self.mat)
        self.assertNotIn('physics',self.lc.bindings)
        self.assertIs(self.left.bindings['physics'],self.mat)
        self.assertEqual(self.robot.gripper_contact_material_diagnostics['collider_bindings'][0]['binding_prim'],self.left.path)
    def test_missing_disabled_or_duplicate_finger_collision_is_rejected(self):
        self.lc.enabled=False
        with self.assertRaises(RuntimeError):bind(self.robot,self.stage,self.mat)
        self.lc.enabled=True;Prim('/robot/duplicate/left',parent=self.root)
        with self.assertRaises(RuntimeError):bind(self.robot,self.stage,self.mat)
    def test_created_material_without_resolved_binding_cannot_pass(self):
        Binding.fail_resolution=True
        with self.assertRaises(RuntimeError):bind(self.robot,self.stage,self.mat)

if __name__=='__main__':unittest.main()
