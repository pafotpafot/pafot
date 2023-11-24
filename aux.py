import random
import carla


def generate_next_pos(current_pos):
    if current_pos == 1:
        next_pos = random.choice([1, 2, 8])
    elif current_pos == 8:
        next_pos = random.choice([7, 8, 1])
    else:
        next_pos = random.choice([current_pos, current_pos - 1, current_pos + 1])

    return next_pos

