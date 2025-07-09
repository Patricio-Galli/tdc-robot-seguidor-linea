import random

def generate_signed_random(min_value, max_value):
    value = random.uniform(-max_value, max_value)
    if abs(value) < min_value:
        return generate_signed_random(min_value, max_value)
    return value