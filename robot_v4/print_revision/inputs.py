"""Validate measurement inputs without substituting missing physical evidence."""
import math


def validate_inputs(config):
    if config.get('schema_version')!=1:raise ValueError('Unsupported input schema')
    def finite(value,path):
        if isinstance(value,bool) or value is None:return
        if isinstance(value,(int,float)):
            if not math.isfinite(value):raise ValueError('Non-finite value: '+path)
        elif isinstance(value,list):
            for i,v in enumerate(value):finite(v,path+'.'+str(i))
        elif isinstance(value,dict):
            for key,v in value.items():finite(v,path+'.'+key)
    finite(config,'inputs')
    for name,item in config['hardware'].items():
        if set(item['actual'])!=set(item['nominal']):raise ValueError('Actual/nominal fields differ: '+name)
        catalog=item.get('catalog',{})
        if catalog and (not catalog.get('url') or not catalog.get('source_id') or not set(catalog.get('values',{}))<=set(item['nominal'])):
            raise ValueError('Invalid supplier data: '+name)
        for key,nominal in item['nominal'].items():
            actual=item['actual'][key]
            v=catalog.get('values',{}).get(key,nominal) if actual is None else actual
            if key=='mount_holes_yz_mm':
                if not v or any(not isinstance(p,list) or len(p)!=2 for p in v):raise ValueError('Invalid hole coordinates: '+name)
            elif key in ('dimensions_mm','connector_keepout_mm'):
                if not isinstance(v,list) or len(v)!=3 or any(isinstance(x,bool) or not isinstance(x,(int,float)) or x<=0 for x in v):raise ValueError('Invalid dimensions: '+name+'.'+key)
            elif key=='connector_side':
                if v not in ('front','rear','left','right'):raise ValueError('Invalid connector side: '+name)
            elif isinstance(nominal,(int,float)):
                if isinstance(v,bool) or not isinstance(v,(int,float)):raise ValueError('Expected number: '+name+'.'+key)
                if key not in ('connector_x_offset_mm','boss_projection_mm') and v<=0:raise ValueError('Expected positive number: '+name+'.'+key)
        if name.startswith('drive_'):
            a=item['actual'];n=item['nominal']
            values=catalog.get('values',{})
            d=values.get('shaft_diameter_mm',n['shaft_diameter_mm']) if a['shaft_diameter_mm'] is None else a['shaft_diameter_mm']
            f=values.get('shaft_flat_depth_mm',n['shaft_flat_depth_mm']) if a['shaft_flat_depth_mm'] is None else a['shaft_flat_depth_mm']
            if not 0<f<d/2:raise ValueError('Invalid shaft flat depth: '+name)
    fit=config['fit']
    for key in ('d_bore_diametral_clearance_mm','d_bore_flat_clearance_mm','hinge_diametral_clearance_mm','nut_pocket_clearance_mm'):
        v=fit[key]
        if v is not None and (isinstance(v,bool) or not isinstance(v,(int,float)) or not 0<=v<=1):raise ValueError('Invalid fit allowance: '+key)
    for key in ('insert_hole_diameter_mm','tyre_inner_diameter_mm'):
        v=fit[key]
        if v is not None and (isinstance(v,bool) or not isinstance(v,(int,float)) or v<=0):raise ValueError('Invalid fit dimension: '+key)
    for key,v in config['slicer']['printed_mass_g_by_file'].items():
        if isinstance(v,bool) or not isinstance(v,(int,float)) or v<=0:raise ValueError('Invalid slicer mass: '+key)
    design=config['design']
    # Agreed acceptance cannot be relaxed by editing the input file.
    if design['hinge_min_wall_mm']<1.6 or design['rear_load_min']<0.10 or design['mass_limit_g']>2000 or design['budget_thb']>6000:raise ValueError('Inputs relax agreed acceptance limits')
