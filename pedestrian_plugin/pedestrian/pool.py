import ros

count = 0
def onUpdate(**args):
    global count
    if count == 0:
        name = args['name'] if 'name' in args else "no name"
        ros.info(f"{name} is pooled")
    count = 1
    return args
