from RobotRaconteur.Client import *
import numpy as np

c = RRN.ConnectService('rr+tcp://localhost:63158?service=tesseract')
robot=RRN.ConnectService('rr+tcp://[fe80::6fd7:46d9:60d1:7d1b]:2355/?nodeid=0912818c-8e8f-4022-b31a-63efdc64ad49&service=Universal_Robot')
robot.command_mode=3

start_waypoint = RRN.NewStructure('com.robotraconteur.robotics.planning.JointWaypoint',c)
end_waypoint = RRN.NewStructure('com.robotraconteur.robotics.planning.JointWaypoint',c)

start_waypoint.joint_positions = np.ones((6,))*.1
start_waypoint.joint_positions[0]=0.537#0.3
start_waypoint.joint_positions[1]=-1.004#-1.0
start_waypoint.joint_positions[2]=-0.067#0.2
start_waypoint.joint_positions[3]=-1.349#1.5
start_waypoint.joint_positions[4]=-0.169#0.2
start_waypoint.joint_positions[5]=-0.0616#0.2
#start_waypoint.joint_positions[6]=0.0#0.2]]
start_waypoint.coeffs = np.ones((1,))
start_waypoint.is_critical=True

end_waypoint.joint_positions = np.ones((6,))*.01
end_waypoint.joint_positions[0]=0.0#0.3
end_waypoint.joint_positions[1]=0.0#-1.0
end_waypoint.joint_positions[2]=0.0#0.2
end_waypoint.joint_positions[3]=0.0#1.5
end_waypoint.joint_positions[4]=0.0#0.2
end_waypoint.joint_positions[5]=0.0#0.2
#end_waypoint.joint_positions[6]=0.0#0.2
print(end_waypoint.joint_positions)
end_waypoint.coeffs = np.ones((1,))
end_waypoint.is_critical=True

planning_request = RRN.NewStructure('com.robotraconteur.robotics.planning.PlanningRequest',c)
planning_request.device = RRN.NewStructure('com.robotraconteur.identifier.Identifier',c)
planning_request.device.name = "manipulator"
uuid_dt = RRN.GetNamedArrayDType('com.robotraconteur.uuid.UUID',c)
planning_request.device.uuid=np.zeros((1,), uuid_dt)
box_dt = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Box',c)
bounds = np.zeros((1,),box_dt)
bounds[0]["origin"]["x"] = -10
bounds[0]["origin"]["y"] = -10
bounds[0]["origin"]["z"] = -10
bounds[0]["size"]["width"] = 20
bounds[0]["size"]["height"] = 20
bounds[0]["size"]["depth"] = 20
planning_request.workspace_bounds = bounds
planning_request.collision_check=False
planning_request.collision_safety_margin=0.25

planning_request.start_waypoint = RR.RobotRaconteurVarValue(start_waypoint, "com.robotraconteur.robotics.planning.JointWaypoint")
planning_request.goal_waypoint = RR.RobotRaconteurVarValue(end_waypoint, "com.robotraconteur.robotics.planning.JointWaypoint")

plan_generator = c.plan(planning_request)
res = plan_generator.Next()
plan_generator.Close()
print(dir(res))
genny=robot.execute_trajectory(res.joint_trajectory)
for i in res.joint_trajectory.waypoints:
        
	out=genny.Next()
out=genny.Next()
genny.Close()
joint_trajectory=res.joint_trajectory
print(res.joint_trajectory)
