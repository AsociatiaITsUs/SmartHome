import json
from lxml import html
import requests
import socket
import os
import socket    
import multiprocessing
import subprocess
import os


def pinger(job_q, results_q):
    """
    Do Ping
    :param job_q:
    :param results_q:
    :return:
    """
    DEVNULL = open(os.devnull, 'w')
    while True:

        ip = job_q.get()

        if ip is None:
            break

        try:
            subprocess.check_call(['ping', '-c1', ip],
                                  stdout=DEVNULL)
            results_q.put(ip)
        except:
            pass


def get_my_ip():
    """
    Find my IP address
    :return:
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return ip


def map_network(pool_size=255):
    """
    Maps the network
    :param pool_size: amount of parallel ping processes
    :return: list of valid ip addresses
    """

    ip_list = list()

    # get my IP and compose a base like 192.168.1.xxx
    ip_parts = get_my_ip().split('.')
    base_ip = ip_parts[0] + '.' + ip_parts[1] + '.' + ip_parts[2] + '.'

    # prepare the jobs queue
    jobs = multiprocessing.Queue()
    results = multiprocessing.Queue()

    pool = [multiprocessing.Process(target=pinger, args=(jobs, results)) for i in range(pool_size)]

    for p in pool:
        p.start()

    # cue the ping processes
    for i in range(1, 255):
        jobs.put(base_ip + '{0}'.format(i))

    for p in pool:
        jobs.put(None)

    for p in pool:
        p.join()

    # collect he results except the router's one.
    while not results.empty():
        ip = results.get()
	if ip != get_my_ip():
		ip_list.append(ip)

    return ip_list


def parseJson(jsonContent):
	y = json.loads(jsonContent)
	print y["name"]

#get all ip addresses in my private network
list = map_network()
print list
# parse list with IPs and read the initial json post in order to identify the IoT devices:

deviceNr = 0

while deviceNr < len(list):
	deviceAddress = "http://"+list[deviceNr]
	#print deviceAddress
	try:
		page = requests.get(deviceAddress)
	except request.ConnectionError, e:
		print e 
	#tree = html.fromstring(page.content)
	#print(page.content)
	#parseJson(page.content)
	fileName = "Device"+str(deviceNr)
	f = open(fileName,"w+")
	f.write(page.content)
	f.close()
	with open(fileName) as json_file:  
		data = json.load(json_file)
		#for p in data['name']:
		print data[0]["name"]
	#y = json.loads(page.content)
	#print(y["name"])
	deviceNr+=1
