from fastapi import FastAPI

app = FastAPI()

@app.get("/")
async def read_root():
    return {"message": "Hello from FastAPI backend!"}

@app.get("/embodied-intelligence")
async def get_embodied_intelligence_status():
    return {"status": "Embodied intelligence module is active!"}

@app.get("/foundations")
async def get_foundations_status():
    return {"status": "Physical AI foundations endpoint active!"}

@app.get("/sensor-data")
async def get_sensor_data():
    # In a real scenario, this would return simulated or actual sensor data
    return {"temperature": 25.5, "humidity": 60, "pressure": 1012, "unit": "hPa"}

@app.get("/ros2-fundamentals")
async def get_ros2_fundamentals_status():
    return {"status": "ROS 2 Fundamentals endpoint active!"}

@app.get("/ros2-nodes-topics-services")
async def get_ros2_nodes_topics_services_status():
    return {"status": "ROS 2 Nodes, Topics, Services endpoint active!"}

@app.get("/ros2-actions-parameters")
async def get_ros2_actions_parameters_status():
    return {"status": "ROS 2 Actions and Parameters endpoint active!"}

@app.get("/gazebo-simulation")
async def get_gazebo_simulation_status():
    return {"status": "Gazebo Simulation endpoint active!"}

@app.get("/unity-robotics")
async def get_unity_robotics_status():
    return {"status": "Unity Robotics endpoint active!"}

@app.get("/urdf-xacro-modeling")
async def get_urdf_xacro_modeling_status():
    return {"status": "URDF & XACRO Modeling endpoint active!"}

@app.get("/isaac-sim-overview")
async def get_isaac_sim_overview_status():
    return {"status": "NVIDIA Isaac Sim Overview endpoint active!"}

@app.get("/synthetic-data-generation")
async def get_synthetic_data_generation_status():
    return {"status": "Synthetic Data Generation endpoint active!"}

@app.get("/perception-manipulation")
async def get_perception_manipulation_status():
    return {"status": "Perception and Manipulation endpoint active!"}

@app.get("/reinforcement-learning")
async def get_reinforcement_learning_status():
    return {"status": "Reinforcement Learning in Isaac Lab endpoint active!"}

@app.get("/vision-language-action")
async def get_vision_language_action_status():
    return {"status": "Vision-Language-Action Models endpoint active!"}

@app.get("/capstone-project")
async def get_capstone_project_status():
    return {"status": "Capstone Project Guidance endpoint active!"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
