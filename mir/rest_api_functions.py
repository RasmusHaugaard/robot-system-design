import requests
import json
import time

def get_mission_guid(mission_name):         
    url = 'http://mir.com/api/v2.0.0/missions'
    token = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
    r = requests.get(url, headers={'Authorization': token}) #stackoverflow: Python request how to pass auth header
    if(r.status_code == 200):   
        r_json = r.json()
        for elem in r_json:
            if(elem['name'] == mission_name):
                print("Found mission: " + mission_name + " - guid: " + elem['guid'] )
                guid = elem['guid']
                return guid 
    else:
        print("Could not find mission: " + mission_name)
        print(r.status_code)
        print(r.content)

"""Returns an integer (id)"""
def add_to_mission_queue(guid):
    token = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
    url = "http://mir.com/api/v2.0.0/mission_queue"

    body = {"mission_id": guid}

    r = requests.post(url, headers={'Authorization': token, 'content-type':'application/json'},
                     data=json.dumps(body)) 
    if r.status_code == 201: #201 - The element has been created successfully
        r_json = r.json()
        print("Posted to mission queue:  " + "-- guid: " +  guid + " -- id: " + str(r_json['id']))
        return r_json['id']
    else:
        print("Could not post to mission queue")
        print(r.status_code)
        print(r.content)


"""Input: the id, which was created when 
   the mission was posted to the queue"""
def remove_mission(id):
    token = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
    url = "http://mir.com/api/v2.0.0/mission_queue/" + str(id)

    r = requests.delete(url, headers={'Authorization' : token})

    if(r.status_code == 204): #The element has been successfully deleted
        print("Succesfully removed: " + str(id) + "from mission queue")
    else:
        print("Could not remove mission from queue")
        print(r.status_code)
        print(r.content)

def get_status():
    url = "http://mir.com/api/v2.0.0/status"
    token = 'Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
    r = requests.get(url, headers={'Authorization' : token})

    if(r.status_code == 200):
        r_json = r.json()
        #print(r_json)
        return r_json
    else:
        print("Could not get status")


guid = get_mission_guid("shp_mission")
queue_id = add_to_mission_queue(guid)

time.sleep(5)
remove_mission(queue_id)

# while True:
#     time.sleep(1)
#     r_json = get_status()
#     print("mode: " + str( r_json['mode_id'] ))
#     print("state_text: " +  r_json['state_text'])