import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped


def normalize_angle(angle):
    res = angle
    while res > math.pi:
        res -= 2.0 * math.pi
    while res < -math.pi:
        res += 2.0 * math.pi
    return res


# [originX, originY]: defining the bottom left corner of the grid
# (in world coordinates);
# resolution: the (square) size of each grid cell (world units);
# [sizeX, sizeY]: the number of grid cells in each dimension.
# Convert world coordinate to grid coordinate.
def to_grid(x, y, originX, originY, sizeX, sizeY, resolution):
    if x < originX or y < originY:
        ans = None
    else:
        [distX, distY] = [abs(x - originX), abs(y - originY)]
        [gx, gy] = [int(distX / resolution), int(distY / resolution)]
        if gx >= sizeX or gy >= sizeY:
            ans = None
        else:
            ans = (gx, gy)
    return ans


# Convert grid coordinate to world coordinate.
def to_world(gx, gy, originX, originY, sizeX, sizeY, resolution):
    if gx >= sizeX or gy >= sizeY:
        ans = None
    else:
        x = originX + gx * resolution
        y = originY + gy * resolution
        ans = (x, y)
    return ans


# Convert grid coordinate to map index.
def to_index(gx, gy, sizeX):
    index = gx + gy * sizeX
    return index


# Given two integer coordinates use Bresenham's line algorithm
# to return a list of coordinates of a line between the two points.
def bresenham(x0, y0, x1, y1):
    # Setup initial conditions
    dx = x1 - x0
    dy = y1 - y0
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
    # Rotate line
    if is_steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
        swapped = True
    # Recalculate differentials
    dx = x1 - x0
    dy = y1 - y0
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y0 < y1 else -1
    # Iterate over bounding box generating points between start and end
    y = y0
    points = []
    for x in range(x0, x1 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


def set_transform_to_laser(resolution):
    dimension = 20.0
    laser_x_offset = 0.0
    laser_y_offset = 0.0
    laser_theta_offset = math.radians(0)
    size = int(dimension / resolution)
    origin = dimension / 2
    q = quaternion_from_euler(0, 0, laser_theta_offset)
    to_laser = TransformStamped()
    to_laser.header.frame_id = '/base_link'
    to_laser.child_frame_id = '/base_laser_link'
    to_laser.transform.translation.x = laser_x_offset
    to_laser.transform.translation.y = laser_y_offset
    to_laser.transform.rotation.x = q[0]
    to_laser.transform.rotation.y = q[1]
    to_laser.transform.rotation.z = q[2]
    to_laser.transform.rotation.w = q[3]
    return [to_laser, size, origin]


# Estimate mu and sigma for the coverage map using what we
# get from the robot, dist is the distance from the robot
# to the center of the cell, z is the measurement.
def estimate_coverage(dist, z, resolution):
    # sigma = z / 5
    sigma = 0.2
    x = dist - z
    if x < -resolution / 2:
        mu = 0
    elif x > resolution / 2:
        mu = 1
    else:
        mu = 1 / 2 + x / resolution
    return [mu, sigma]


def compute_entropy(p1, p2, p3):
    [p1, p2, p3] = [max(p1, 0.001), max(p2, 0.001), max(p3, 0.001)]
    part1 = p1 * math.log(p1)
    part2 = p2 * math.log(p2)
    part3 = p3 * math.log(p3)
    entropy = round(-(part1 + part2 + part3), 4)
    return entropy


def wrapped_Gaussian(p, mu, sigma):
    pi = math.pi
    sumK = 0
    theta = pi * p
    mu = pi * mu
    sigma = pi * sigma
    for k in range(1, 500):
        term1 = math.exp(-(theta - mu + 2 * pi * k)**2 / (2 * sigma**2))
        term2 = math.exp(-(theta - mu - 2 * pi * k)**2 / (2 * sigma**2))
        sumK = sumK + term1 + term2
    sumK = sumK + math.exp(-(theta - mu)**2 / (2 * sigma**2))
    pdf = (1 / (sigma * math.sqrt(2 * math.pi))) * sumK
    return pdf


def normalize(samples):
    samples = [sample / sum(samples) for sample in samples]
    return samples


def wrapped_Gaussian_sampling(x_samples, mu, sigma):
    y_samples = []
    for x_sample in x_samples:
        y_sample = wrapped_Gaussian(x_sample, mu, sigma)
        y_samples.append(y_sample)
    y_samples = normalize(y_samples)
    y_samples = [round(y, 3) for y in y_samples]
    return y_samples


def update_estimation(samples1, samples2):
    samples = []
    for i in range(len(samples1)):
        sample1 = samples1[i]
        sample2 = samples2[i]
        sample3 = sample1 * sample2
        samples.append(sample3)
    samples = normalize(samples)
    samples = [round(sample, 3) for sample in samples]
    return samples


if __name__ == "__main__":
    # dist is the distance from the robot to the center of
    # the cell, z is the measurement.
    resolution = 0.1
    x_samples = [0.1, 0.5, 0.9]
    [dist, z] = [1.0, 0.96]
    [mu, sigma] = estimate_coverage(dist, z, resolution)
    y0_samples = wrapped_Gaussian_sampling(x_samples, mu, sigma)
    entropy0 = compute_entropy(y0_samples[0], y0_samples[1], y0_samples[2])
    y1_samples = y0_samples
    samples = update_estimation(y0_samples, y1_samples)
    entropy1 = compute_entropy(y1_samples[0], y1_samples[1], y1_samples[2])
    print('Distribution: {}, entropy = {};'.format(y0_samples, entropy0))
    print('Updated, distribution: {}, entropy = {}'.format(y1_samples, entropy1))
