"""Storage admission only; never changes physics or assembly acceptance."""
GIB=1024**3

def startup_memory_required(gib=20, shared=False, *, batch=None, round_number=None):
    if type(gib) is not int or gib<12 or (gib<20 and shared is not True):
        raise ValueError('Reduced memory admission requires shared permission and at least12GiB')
    if gib<16:
        batch=batch if isinstance(batch,dict) else {}
        authorization=batch.get('startup_memory_authorization') or {}
        rounds=batch.get('round_numbers')
        valid_rounds=(isinstance(rounds,list) and len(rounds)==2
            and all(type(x) is int for x in rounds) and rounds[0]<=rounds[1])
        if not (isinstance(authorization,dict)
                and authorization.get('authorized') is True
                and authorization.get('scope')=='current_batch_only'
                and authorization.get('minimum_gib')==12
                and valid_rounds and authorization.get('batch_round_numbers')==rounds
                and type(round_number) is int and rounds[0]<=round_number<=rounds[1]
                and isinstance(authorization.get('human_instruction'),str)
                and authorization['human_instruction'].strip()):
            raise ValueError('Below16GiB requires explicit matching current-batch12GiB authorization')
    return gib*GIB

def startup_required(prefix,full,reserve):
    if not (0<=prefix<=full and full>0 and reserve>0):raise ValueError('Invalid output budget')
    return (prefix or full)+reserve

def check_budget(free,written,phase,checkpoint,prefix,full,reserve):
    startup_required(prefix,full,reserve)
    if written<0 or free<0:raise ValueError('Negative disk accounting')
    full_due=bool(prefix and ((phase is not None and phase>=checkpoint) or written>=prefix))
    remaining=max(0,(full if full_due else prefix or full)-written)
    required=reserve+remaining
    return {'full_due':full_due,'required_free_bytes':required,'passed':free>=required,
            'observed_free_bytes':free,'output_bytes':written,'phase_index':phase}
