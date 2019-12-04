import requests
import json
import time
from rsd import conf

BASE_URL = conf.MIR_SERVER_URL + "/api/v2.0.0"
#BASE_URL = "http://mir.com/api/v2.0.0"
token = "Basic UlNEMjAxOTo0NWVkZWQ4YmRjMjJmZGQ5ZjcwOTJmY2RhMDhkOWFmMjdjNDNhNjQ4MWFkZDYyMDgwZWI3MmM0OTcyOTQ1NzZl"


def get_mission_guid(mission_name):
    url = BASE_URL + '/missions'
    r = requests.get(url, headers={'Authorization': token})  # stackoverflow: Python request how to pass auth header
    if r.status_code == 200:
        r_json = r.json()
        for elem in r_json:
            if (elem['name'] == mission_name):
                print("Found mission: " + mission_name + " - guid: " + elem['guid'])
                guid = elem['guid']
                return guid
    else:
        print("Could not find mission: " + mission_name)
        print(r.status_code)
        print(r.content)


"""Returns an integer (id)"""

def add_to_mission_queue(guid):
    url = BASE_URL + "/mission_queue"

    body = {"mission_id": guid}

    r = requests.post(url, headers={'Authorization': token, 'content-type': 'application/json'},
                      data=json.dumps(body))
    if r.status_code == 201:  # 201 - The element has been created successfully
        r_json = r.json()
        print("Posted to mission queue:  " + "-- guid: " + guid + " -- id: " + str(r_json['id']))
        return r_json['id']
    else:
        print("Could not post to mission queue")
        print(r.status_code)
        print(r.content)


"""Input: the id, which was created when 
   the mission was posted to the queue"""

def remove_mission(id):
    url = BASE_URL + "/mission_queue/" + str(id)

    r = requests.delete(url, headers={'Authorization': token})

    if r.status_code == 204:  # The element has been successfully deleted
        print("Succesfully removed: " + str(id) + "from mission queue")
    else:
        print("Could not remove mission from queue")
        print(r.status_code)
        print(r.content)


def get_register_value(register_id):
    url = BASE_URL + "/registers"
    r = requests.get(url, headers={'Authorization': token})

    if r.status_code == 200:
        r_json = r.json()
        for elem in r_json:
            if elem['id'] == register_id:
                value = elem['value']
        print("Register " + str(register_id) + ": " + str(value))
        return value
    else:
        print(r.status_code)
        print(r.content)


def set_register_value(register_id, value):
    url = BASE_URL + "/registers/" + str(register_id)
    body = {"value": value, "label": ""}
    r = requests.put(url, headers={'Authorization': token, 'content-type': 'application/json'}, data=json.dumps(body))

    if r.status_code == 200:
        print("Set register " + str(register_id) + ": " + str(value))
    else:
        print(r.status_code)
        print(r.content)


def get_status():
    url = BASE_URL + "/status"
    r = requests.get(url, headers={'Authorization': token})

    if r.status_code == 200:
        r_json = r.json()
        # print(r_json)
        return r_json
    else:
        print("Could not get status")




def get_mission_queue():
    url = BASE_URL + "/mission_queue"
    r = requests.get(url, headers={'Authorization': token})
    if r.status_code == 200:
        print("get_mission_queue reponse: %s", r.json())
        return r.json()
    else:
        print("Could not retrieve mission queue")
        print(r.status_code)
        print(r.content)

"""Uses the id"""
def get_mission_info_by_id(id):
    url = BASE_URL + "/mission_queue/" + str(id)
    r = requests.get(url, headers={'Authorization': token})
    if r.status_code == 200:
        return r.json()
    else:
        print(r.status_code)
        print(r.content)








# guid = get_mission_guid("G9_mission")
# id1 = add_to_mission_queue(guid)
# mission_queue = get_mission_queue()

# print(mission_queue[654])

# mq = get_mission_queue()
# l = []
# for d in mq:
#     state = d['state']
#     if state == 'Pending' or state == 'Executing':
#         print("Found a pending/executing mission")
#         l.append( d['id'] )
# print(l)
# for id in l:
#     m = get_mission_info_by_id(id)
#     if m['mission_id'] == get_mission_guid("G9_mission"):
#         print("gotcha")
#         remove_mission(id)







# id2 = add_to_mission_queue(guid)

# print("id1: %2d \n id2: %2d \n" % (id1, id2) )



# set_register_value(9,101)
# get_register_value(9)
# set_register_value(9,1337)
# get_register_value(9)


# guid = get_mission_guid("shp_mission")
# queue_id = add_to_mission_queue(guid)

# while True:
#     time.sleep(1)
#     r_json = get_status()
#     print("mode: " + str( r_json['mode_id'] ))
#     print("state_text: " +  r_json['state_text'])
#     get_register_value(9)
