import requests
import json
import time

url = "http://localhost:8001/chat"
headers = {"Content-Type": "application/json"}
payload = {"question": "What is ROS 2?"} # Changed from 'query' to 'question'

print(f"Sending POST request to {url} with payload: {json.dumps(payload, indent=2)}")

try:
    response = requests.post(url, headers=headers, json=payload, timeout=30)
    response.raise_for_status()  # Raise an exception for HTTP errors (4xx or 5xx) 
    
    response_json = response.json()
    print("\nReceived response:")
    print(json.dumps(response_json, indent=2))

    # Basic assertions to check response structure
    assert "answer" in response_json
    assert "sources" in response_json
    assert isinstance(response_json["answer"], str)
    assert isinstance(response_json["sources"], list)

    print("\nTest passed: Response structure is as expected.")
except requests.exceptions.RequestException as exc:
    print(f"An error occurred while requesting {exc.request.url!r}: {exc}")
except requests.exceptions.HTTPStatusError as exc:
    print(f"Error response {exc.response.status_code} while requesting {exc.request.url!r}: {exc.response.text}")
except AssertionError as exc:
    print(f"Test failed: {exc}")
except Exception as exc:
    print(f"An unexpected error occurred: {exc}")
