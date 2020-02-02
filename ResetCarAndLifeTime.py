import numpy
# Outputs
# Reset Car (location + heading + steeringAngle), Lifetime, and prev_carLines
def ResetCarAndLifeTime (carLocations, env, car_id, carHeadings, steerAngles,LifeTimes,prev_carLines):
    carLocations = env.start_points
    carHeadings  = env.start_headings

    # carLocations(car_id, :) = ((num-1-num2)*rand(1,2)+1+num2/2);
    # carHeadings(car_id)  = (180*rand-90) * pi/180;

    steerAngles = env.start_steerAngles

    LifeTimes = 0

    for i in range(len(prev_carLines)):
        for j in range(len(prev_carLines[i])):
            prev_carLines[i][j]= 0
