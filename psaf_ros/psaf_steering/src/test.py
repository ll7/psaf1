
import numpy as np


def normalize_meter(x):
    normalized = (x[0] + x[1] * 256 + x[2] * 256 * 256) / (256 * 256 * 256 - 1)
    in_meters = 1000 * normalized
    return in_meters

v_normalize_meter = np.vectorize(normalize_meter)

mapping = [1 / (256 * 256 * 256 - 1) * 1000, 256 / (256 * 256 * 256 - 1) * 1000, 256 * 256 / (256 * 256 * 256 - 1) * 1000] 


# normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
# in_meters = 1000 * normalized


image_data = b'aaaabbbbaaaabbbb'


array = np.frombuffer(image_data, dtype=np.dtype("uint8"))
array = np.reshape(array, (2, 2, 4))

vec = v_normalize_meter(array)


print(array)
print(vec)