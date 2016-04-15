import csv

# This script reads in a csv file and creates a yaml file of all the apriltags.
# Update database and dowload csv from https://docs.google.com/spreadsheets/d/1vvrkYaFktDBXyF4E_MMxx3wTX5U5S1ToQBbKSQ6uVoA/edit#gid=2078462339

def tag_writer(id, tag_type,street_name, vehicle_name, sign_type, location_226, location_316):
    yaml.write('- tag_id: ' + str(id) + '\n')
    yaml.write('  tag_type: ' + tag_type + '\n')
    yaml.write('  street_name: ' + street_name + '\n')
    yaml.write('  vehicle_name: ' + vehicle_name + '\n')
    yaml.write('  traffic_sign_type: ' + sign_type + '\n')
    yaml.write('  location_226: ' + location_226 + '\n')
    yaml.write('  location_316: ' + location_316 + '\n')
    yaml.write('\n')

yaml = open('apriltagsDB.yaml', 'w')

n = 0
N = 1
with open('tagsDB.csv', 'rb') as csvfile:
    tagDB = csv.reader(csvfile, delimiter=',')
    for row in tagDB:
        if N < 4:
            N += 1
        else:
            while True:
                if int(row[0]) == n:
                    tag_writer(row[0], row[1], row[2], row[3], row[4], row[5], row[6])
                    n += 1
                    break
                else:
                    tag_writer(n, '', '', '', '', '', '')
                    n += 1
