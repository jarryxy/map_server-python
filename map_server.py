import numpy as np
from PIL import Image
import tf.transformations as tf_trans
from geometry_msgs.msg import Quaternion
import rospy
from nav_msgs.msg import OccupancyGrid
import yaml


class map_resp_():
    def __init__(self) -> None:
        self.map = OccupancyGrid()


def loadMapFromYaml(path_to_yaml) -> map_resp_:
    try:
        with open(path_to_yaml, 'r') as file:
            map_data = yaml.safe_load(file)
    except Exception:
        print(f'Map_server could not open {path_to_yaml}')
        return None

    try:
        resolution = map_data['resolution']
    except KeyError:
        print('The map does not contain a resolution tag or it is invalid.')
        return None
    try:
        negate = map_data['negate']
    except KeyError:
        print('The map does not contain a negate tag or it is invalid.')
        return None
    try:
        occupied_thresh = map_data['occupied_thresh']
    except KeyError:
        print('The map does not contain an occupied_thresh tag or it is invalid.')
        return None
    try:
        free_thresh = map_data['free_thresh']
    except KeyError:
        print('The map does not contain a free_thresh tag or it is invalid.')
        return None
    try:
        image = map_data['image']
    except KeyError:
        print('The map does not contain an image tag or it is invalid.')
        return None
    try:
        mode = map_data['mode']
        if mode not in ('trinary', 'scale', 'raw'):
            print(f'Invalid mode tag {mode}')
            return None
    except KeyError:
        mode = 'trinary'
        print('The map does not contain a mode tag or it is invalid... assuming Trinary')
    try:
        origin = map_data['origin']
        if len(origin) != 3:
            print('The map does not contain an origin tag or it is invalid.')
            return None
    except Exception:
        print('The map does not contain an origin tag or it is invalid.')
        return None
    map_resp = map_resp_()
    load_map_from_file(map_resp, image, resolution, negate, occupied_thresh, free_thresh, origin, mode)
    map_resp.map.info.map_load_time = rospy.get_rostime()
    map_resp.map.header.stamp = rospy.get_rostime()
    map_resp.map.header.frame_id = "map"
    print(f'Read a {map_resp.map.info.width} X {map_resp.map.info.height} map @ {map_resp.map.info.resolution} m/cell')
    return map_resp


def load_map_from_file(resp, fname, res, negate, occ_th, free_th, origin, mode='trinary'):
    img = Image.open(fname)

    # Copy the image data into the map structure
    resp.map.info.width = img.width
    resp.map.info.height = img.height
    resp.map.info.resolution = res
    resp.map.info.origin.position.x = origin[0]
    resp.map.info.origin.position.y = origin[1]
    resp.map.info.origin.position.z = 0.0
    quaternion = tf_trans.quaternion_from_euler(0, 0, origin[2])
    resp.map.info.origin.orientation = Quaternion(*quaternion)

    # Allocate space to hold the data
    resp.map.data = np.zeros(resp.map.info.width * resp.map.info.height, dtype=np.int8)

    # Copy pixel data into the map structure
    pixels = np.array(img)
    for j in range(resp.map.info.height):
        for i in range(resp.map.info.width):
            color_avg = np.mean(pixels[j, i])

            if negate:
                color_avg = 255 - color_avg

            if mode == "raw":
                value = int(color_avg)
                resp.map.data[MAP_IDX(resp.map.info.width, i, resp.map.info.height - j - 1)] = value
                continue

            occ = (255 - color_avg) / 255.0

            if occ > occ_th:
                value = +100
            elif occ < free_th:
                value = 0
            elif mode == "trinary":
                value = -1
            else:
                ratio = (occ - free_th) / (occ_th - free_th)
                value = int(1 + 98 * ratio)

            resp.map.data[MAP_IDX(resp.map.info.width, i, resp.map.info.height - j - 1)] = value

    img.close()


def MAP_IDX(width, x, y):
    return y * width + x


if __name__ == '__main__':
    rospy.init_node('map_server', anonymous=True)
    map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=1)
    # 读取mymap.yaml文件
    file_path = '/home/mymap.yaml'
    resp = loadMapFromYaml(file_path)
    print(resp.map)
    # rate = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     map_publisher.publish(resp.map)
    #     rate.sleep()
