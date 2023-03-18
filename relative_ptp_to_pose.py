import math

def relative_ptp(ix, iy, t, distance):
    x_error = math.sin(math.radians(t)) * distance
    y_error = math.cos(math.radians(t)) * distance

    x = x_error + x;
    y = y_error + y;
    print(x, y)

relative_ptp()