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


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
