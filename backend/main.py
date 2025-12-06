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


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
