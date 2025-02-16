

def read_param(sim,objectHandle,name:str,default=None):
    data = sim.getBufferProperty(objectHandle,"customData."+name,{'noError':True})
    if data:    
        return sim.unpackTable(data)
    else:
        return default  
    
def write_param(sim,objectHandle,name:str,data:dict):
    data_new = read_param(sim,objectHandle,name,default={})
    
    for key in data:
        data_new[key] = data[key]

    sim.setBufferProperty(objectHandle,"customData."+name,sim.packTable(data_new))
