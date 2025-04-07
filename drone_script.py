from dronekit import connect, VehicleMode
from geopy.distance import geodesic
import time
import math

vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)


def get_alt():
    return vehicle.location.global_relative_frame.alt


def send_rc_override(channel, pwm):
    vehicle.channels.overrides[channel] = pwm


def get_direction(lat1, lon1, lat2, lon2):
    dLon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    x = math.sin(dLon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    )
    initial_direction = math.atan2(x, y)
    direction = (math.degrees(initial_direction) + 360) % 360
    return direction


def get_distance_meters(lat1, lon1, lat2, lon2):
    coord1 = (lat1, lon1)
    coord2 = (lat2, lon2)
    return geodesic(coord1, coord2).meters


def throttle(alt):
    alt = 100
    pre_alt = alt - alt / 100 * 4

    if get_alt() < pre_alt:
        send_rc_override(3, 2000)
    elif get_alt() < alt:
        send_rc_override(3, 1500)
    else:
        send_rc_override(3, 1400)


def runner(distance):
    if (distance > 100):
        send_rc_override(2, 1000)
    elif distance < 100 and distance > 20:
        send_rc_override(2, 1400)
    elif distance < 20 and distance > 4:
        send_rc_override(2, 1475)
    else:
        send_rc_override(2, 1500)


def set_direction(target_direction, current_direction, tolerance):
    turn_angle = (target_direction - current_direction + 360) % 360
    pwm = 1500
    if turn_angle > 180:
        turn_angle -= 360

    if abs(current_direction - target_direction) > tolerance:
        diff = int(abs(current_direction - target_direction) / 30)
        if diff < 1:
            diff = 1
        diff *= 30
        if turn_angle < 0:
            pwm -= diff
        else:
            pwm += diff

    send_rc_override(4, pwm)


while not vehicle.mode.name == 'ALT_HOLD':
    print("Чекаємо на зміну режиму AltHold!")
    vehicle.mode = VehicleMode("ALT_HOLD")
    time.sleep(1)


while not vehicle.armed:
    print("Чекаємо на армування!")
    vehicle.armed = True
    time.sleep(1)

print("Армовано в AltHold режимі")

vehicle.channels.overrides = {'1': 1500, '3': 1000, '2': 1500,
                              '5': 1800, '4': 1500, '7': 1000, '6': 1000, '8': 1800}

list_points = []
list_points.append({
    'target_alt': 100,
    'lat': 50.443326,
    'lon': 30.448078,
    'target_distance': 4,
    'passed': False
})

list_points.append({
    'target_alt': 100,
    'azimuth': 350,
    'passed': False
})

try:
    for point in list_points:
        print(
            f"Виконуємо місію № {list_points.index(point) + 1}: Параметри місії: {point}")
        while not point.get('passed'):
            current_direction = vehicle.heading

            if not point.get('azimuth'):
                current_location = vehicle.location.global_relative_frame
                lat = current_location.lat
                lon = current_location.lon
                target_direction = get_direction(
                    lat, lon, point.get('lat'), point.get('lon'))
                distance = get_distance_meters(
                    lat, lon, point.get('lat'), point.get('lon'))
                tolerance = int(distance / 400)
                if tolerance < 1:
                    tolerance = 0.5

                if abs(get_alt() - point.get('target_alt')) < 4:
                    print(f"Дистанція до цілі: {int(distance)} (м)")
                    runner(distance)

                if distance < point.get('target_distance'):
                    point['passed'] = True
            else:
                target_direction = point.get('azimuth')
                tolerance = 3

                if abs(current_direction - target_direction) <= tolerance:
                    point['passed'] = True

            throttle(point.get('target_alt'))

            set_direction(target_direction, current_direction, tolerance)

            time.sleep(1)
        print(f"Місію № {list_points.index(point) + 1} виконано!")

    print("Посадка дрона!")
    while get_alt() > 1:
        if get_alt() < 15:
            send_rc_override(3, 1300)
            if vehicle.velocity[2] == 0:
                break
        time.sleep(1)
    print("Дрон посадженно!")
except KeyboardInterrupt:
    print("Зупинка користувачем")

vehicle.channels.overrides = {}

vehicle.armed = False

vehicle.close()
