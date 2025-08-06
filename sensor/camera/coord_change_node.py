import numpy as np 



def coord_change(x,y,cx,cy):
    dx = x - cx
    dy = y - cy
    r = (np.sqrt( (dx)**2 + (dy)**2 ))
    theta = np.arctan2(dx, -dy)
    theta_deg = (np.degrees(theta))
    return round(float(r),2),round(float(theta_deg),2)

# print(coord_change(x,y,cx,cy)) >> for


