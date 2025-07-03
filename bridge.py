import traci
import json
import math
import os

UWB_RANGE = 50.0  # meters
STEP_SIZE = 0.1   # SUMO simulation step in seconds

vehicle_ids = ["t_0", "t_1", "t_2", "t_3"]
pedestrian_id = "p_0"
output_file = "uwb_events.json"

def euclidean_dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def main():
    sumo_cmd = ["sumo", "-c", "sumotestdrive.sumocfg"]  # replace with your actual config
    traci.start(sumo_cmd)

    events = []
    sim_time = 0.0

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        try:
            ped_pos = traci.person.getPosition(pedestrian_id)
        except traci.TraCIException:
            ped_pos = None

        if ped_pos:
            for veh_id in vehicle_ids:
                try:
                    car_pos = traci.vehicle.getPosition(veh_id)
                    distance = euclidean_dist(car_pos, ped_pos)
                    if distance <= UWB_RANGE:
                        event = {
                            "time": round(sim_time, 2),
                            "sender_id": veh_id,
                            "receiver_id": pedestrian_id,
                            "sender_position": [round(car_pos[0], 2), round(car_pos[1], 2)],
                            "receiver_position": [round(ped_pos[0], 2), round(ped_pos[1], 2)]
                        }
                        events.append(event)
                except traci.TraCIException:
                    continue

        sim_time += STEP_SIZE

    traci.close()

    # Save to JSON
    with open(output_file, "w") as f:
        json.dump(events, f, indent=2)

    print(f"✅ Generated {len(events)} UWB events in {output_file}")

if __name__ == "__main__":
    main()
