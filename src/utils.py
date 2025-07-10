import math
import random

def generate_signed_random(min_value, max_value):
    value = random.uniform(-max_value, max_value)
    if abs(value) < min_value:
        return generate_signed_random(min_value, max_value)
    return value

def ceil_to_nearest(value, base=10):
    return base * math.ceil(value / base)