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
