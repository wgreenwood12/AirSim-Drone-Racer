import airsimneurips


#connect to server with drone controls
client = airsimneurips.MultirotorClient()
client.confirmConnection()
print('Connection confirmed')

"""
To load the level first comment out the code below and run the exe file
Once the level is loaded comment out the simLoadLevel code and uncomment your code
Run the python script again and you should see the drone takeoff
"""
client.simLoadLevel('Soccer_Field_Easy')
client.simStartRace()

client.enableApiControl(vehicle_name="drone_1")
client.arm(vehicle_name="drone_1")

client.takeoffAsync(vehicle_name="drone_1").join()
# Async methods returns Future. Call join() to wait for task to complete.
client.moveToPositionAsync(20, 20, 20, 20, vehicle_name="drone_1").join()

# Takeoff from the new starting position
