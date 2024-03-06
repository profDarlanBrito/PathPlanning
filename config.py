# Module used to read the configuration file
import yaml
import os

# with open('config.cfg') as f:
#     for line in f.readlines():
#         list_line = line.split(sep=':')
#         if list_line[0] == 'path':
#             if len(list_line) == 3:
#                 complete_path = list_line[1].rstrip() + ":" + list_line[2].rstrip()
#             else:
#                 complete_path = list_line[1].rstrip()
#         if list_line[0] == 'filename':
#             file_name = list_line[1].rstrip()
#         if list_line[0] == 'sequence':
#             image_sequence = [int(x) for x in list_line[1].split(sep=",")]
#         if list_line[0] == 'extension':
#             image_extension = list_line[1].rstrip()
#         if list_line[0] == 'simulation time':
#             Maxtime = float(list_line[1])
#         if list_line[0] == 'sensor name':
#             sensor_name = list_line[1].rstrip()
#         if list_line[0] == 'quadcopter name':
#             quadcopter_name = list_line[1].rstrip()
#         if list_line[0] == 'time to stabilize':
#             time_to_stabilize = float(list_line[1])
#         if list_line[0] == 'max displacement':
#             max_displacement = float(list_line[1])
#         if list_line[0] == 'total simulation time':
#             total_simulation_time = float(list_line[1])
#         if list_line[0] == 'displacement':
#             displacement = [float(x) for x in list_line[1].split(sep=",")]
#         if list_line[0] == 'height':
#             height = int(list_line[1])
#         if list_line[0] == 'width':
#             width = int(list_line[1])
#         if list_line[0] == 'target joint xy':
#             xy_joint_name = list_line[1].rstrip()
#         if list_line[0] == 'target joint zy':
#             zy_joint_name = list_line[1].rstrip()


def parse_settings_file(filename):

    if not os.path.exists(filename):
        print('File does not exist:', filename)
        quit()

    print('Using for calibration settings: ', filename)

    with open(filename) as f:
        settings = yaml.safe_load(f)

    if not settings['is_ok']:
        print('Configuration file is incorrect')
        quit()

    return settings


if __name__ == '__main__':
    parse_settings_file('config.yaml')