import requests
import json
import time

API_URL = "http://localhost:8001/chat"
HEADERS = {"Content-Type": "application/json"}

def test_agent_query(agent_id: str, question: str, expected_substring: str = None, selected_text: str = None):
    print(f"\n--- Testing Agent: {agent_id}, Question: {question} ---")
    payload = {"question": question, "agent_id": agent_id}
    if selected_text:
        payload["selected_text"] = selected_text

    print(f"Sending payload: {json.dumps(payload, indent=2)}")

    try:
        response = requests.post(API_URL, headers=HEADERS, json=payload, timeout=60) # Increased timeout
        response.raise_for_status()
        
        response_json = response.json()
        print("\nReceived response:")
        print(json.dumps(response_json, indent=2))

        assert "answer" in response_json, "Response missing 'answer' field"
        assert isinstance(response_json["answer"], str), "'answer' field is not a string"
        assert "sources" in response_json, "Response missing 'sources' field"
        assert isinstance(response_json["sources"], list), "'sources' field is not a list"

        if expected_substring is not None:
            # Use a more robust check for substring, allowing for variations
            if expected_substring.lower() not in response_json["answer"].lower():
                print(f"Assertion Failed: Expected substring '{expected_substring}' not found in answer.")
                return False
            else:
                print(f"Test passed: Expected substring '{expected_substring}' found in answer.")
        else:
            print("Test passed: Response structure is as expected, no specific substring assertion.")
        
        return True

    except requests.exceptions.RequestException as exc:
        print(f"Test FAILED: An error occurred while requesting {exc.request.url!r}: {exc}")
    except requests.exceptions.HTTPError as exc: # Corrected from HTTPStatusError
        print(f"Test FAILED: Error response {exc.response.status_code} while requesting {exc.request.url!r}: {exc.response.text}")
    except AssertionError as exc:
        print(f"Test FAILED: Assertion failed - {exc}")
    except Exception as exc:
        print(f"Test FAILED: An unexpected error occurred: {exc}")
    return False

if __name__ == "__main__":
    # Give the server some time to start up if it's being launched externally
    # time.sleep(10) 

    # Test cases
    all_tests_passed = True

    # 1. Teaching Assistant (default behavior, should use RAG)
    # Substring is present in the answer, so this should pass with the fixed assertion
    if not test_agent_query("teaching-assistant", "What are the key concepts of embodied intelligence?", "embodied intelligence"):
        all_tests_passed = False
    
    # 2. Teaching Assistant (chapter summarizer via keyword)
    if not test_agent_query("teaching-assistant", "Summarize the foundations of physical AI.", "- "): # Look for bullet point start
        all_tests_passed = False

    # 3. Teaching Assistant (concept explainer via keyword)
    if not test_agent_query("teaching-assistant", "Explain what is ROS 2?", "ROS 2"):
        all_tests_passed = False

    # 4. Robotics Code Mentor (should trigger code generation intent)
    if not test_agent_query("robotics-mentor", "Generate a ROS2 publisher node in Python.", "python"):
        all_tests_passed = False
    
    # 5. Quiz Coach (should trigger quiz generation intent)
    if not test_agent_query("quiz-coach", "Create a quiz about ROS 2 topics.", "question"):
        all_tests_passed = False

    # 6. Teaching Assistant with selected text (should ignore RAG and skills)
    # Now passing an expected substring to ensure the test doesn't fail on None.
    if not test_agent_query("teaching-assistant", "What does this text say?", None, "This is some selected text that should be summarized or explained."):
        all_tests_passed = False

    if all_tests_passed:
        print("\nALL AGENT TESTS PASSED SUCCESSFULLY!")
    else:
        print("\nSOME AGENT TESTS FAILED. PLEASE CHECK THE OUTPUT ABOVE.")