def line_intersection(p1, p2, p3, p4):
    def cross_product(a, b):
        return a[0] * b[1] - a[1] * b[0]
    
    def subtract(a, b):
        return (a[0] - b[0], a[1] - b[1])
    
    r = subtract(p2, p1)
    s = subtract(p4, p3)
    denominator = cross_product(r, s)
    
    if denominator == 0:
        return None
    
    numerator_t = cross_product(subtract(p3, p1), s)
    numerator_u = cross_product(subtract(p3, p1), r)
    t = numerator_t / denominator
    u = numerator_u / denominator
    
    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection_x = p1[0] + t * r[0]
        intersection_y = p1[1] + t * r[1]
        return (intersection_x, intersection_y)
    
    return None

def extend_segment(p1, p2, y_low, y_high):
    x1, y1 = p1
    x2, y2 = p2
    
    if y1 == y2:
        # Horizontal line case: return extended horizontally at y_low and y_high
        return (x1, y_low), (x2, y_high)
        # return p1, p2
    
    # Compute slope
    slope = (x2 - x1) / (y2 - y1)
    
    # Compute new x-coordinates
    x_low = x1 + slope * (y_low - y1)
    x_high = x1 + slope * (y_high - y1)
    
    return (x_low, y_low), (x_high, y_high)