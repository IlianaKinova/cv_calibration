def boxFromWindow(window):
    data = window.get_geometry()
    return {'top':int(data.y), 'left':int(data.x), 'width':int(data.width), 'height':int(data.height)}

def rvizRemoveBorders(box):
    titleBorder = 23
    outsideBorder = 1
    return {
        'top':int(box['top'] + titleBorder),
        'left':int(box['left'] + outsideBorder), 
        'width':int(box['width'] - (2 * outsideBorder)), 
        'height':int(box['height'] - (titleBorder + outsideBorder))}

def getColorStandardizedRect(box):
    ratio = float(box['width'])/box['height']
    if (16.0/9.0) > ratio:
        width = float(box['width'])
        height = float(box['width']) / (16.0/9.0)
    else:
        width = float(box['height']) * (16.0/9.0)
        height = float(box['height'])
    x = box['left'] + float(box['width'])/2.0 - float(width)/2.0
    y = box['top'] + float(box['height'])/2.0 - float(height)/2.0
    return {'top':int(round(y)),'left':int(round(x)),'width':int(round(width)),'height':int(round(height))}

def isolateCamera(window):
    """
    Start from a capture of the entire camera window and isolate the camera with a 16:9 ratio
    """
    return getColorStandardizedRect(rvizRemoveBorders(boxFromWindow(window)))