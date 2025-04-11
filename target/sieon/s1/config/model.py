# Modify this file to decide which model are compiled
from building import *

vehicle_type = GetOption('vehicle')

if vehicle_type == 'Multicopter':
    MODELS = [
        'plant/multicopter',
        'ins/cf_ins',
        'fms/mc_fms',
        'control/mc_controller',
    ]
elif vehicle_type == 'Template':
    MODELS = [
        'plant/template_plant',
        'ins/template_ins',
        'fms/template_fms',
        'control/template_controller',
    ]
elif vehicle_type == 'Boat':
    MODELS = [
        'plant/boat-test',
        'ins/cf_ins',
        'fms/boat_fms',
        'control/boat_controller',
    ]
elif vehicle_type == 'Car_test':
    MODELS = [
        'plant/template_plant',
        'ins/cf_ins',
        'fms/car_test_fms',
        'control/car_test_controller',
    ]
else:
    raise Exception("Wrong VEHICLE_TYPE %s defined" % vehicle_type)