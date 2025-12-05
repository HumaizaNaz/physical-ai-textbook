from fastapi import FastAPI

app = FastAPI()

@app.get("/")
async def read_root():
    return {"message": "Hello from FastAPI backend!"}

@app.get("/embodied-intelligence")
async def get_embodied_intelligence_status():
    return {"status": "Embodied intelligence module is active!"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
