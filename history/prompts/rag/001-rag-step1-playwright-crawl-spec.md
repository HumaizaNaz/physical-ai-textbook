# 001 RAG: Step 1 - Playwright-based Content Crawling Specification

## Context
The "physical-ai-textbook" project requires a robust mechanism for crawling its content for RAG (Retrieval-Augmented Generation) purposes. Previous attempts using standard HTTP `requests` failed due to anti-bot protections (e.g., Cloudflare) on the Vercel-hosted site. Playwright has been identified as the solution to bypass these protections and reliably crawl the textbook content.

## Objective
To successfully crawl all publicly accessible pages of the "physical-ai-textbook" hosted on Vercel, extract clean text content along with page titles and URLs, and store this data in a structured JSON format. This forms the first step in the RAG pipeline.

## Tooling
*   **Playwright:** Used for headless browser automation to mimic human interaction and bypass anti-bot measures.
*   **`requests`:** Used for initially fetching the `sitemap.xml` to get the list of URLs to crawl.
*   **`BeautifulSoup`:** Used for parsing the `sitemap.xml` to extract URLs.
*   **`trafilatura`:** Used for robust content extraction (cleaning HTML into readable text).
*   **`json`:** Used for serializing the extracted data into a JSON file.

## Implementation Details
*   **Location:** The crawling script will be created as `backend_RAG/1_crawl_playwright.py`.
*   **Sitemap Acquisition:** The script first fetches `https://physical-ai-textbook-five.vercel.app/sitemap.xml` to obtain a list of all pages.
*   **Playwright Browser Automation:**
    *   Launches a headless Chromium browser.
    *   Sets a user agent to mimic a typical desktop browser.
    *   Navigates to each URL from the sitemap.
    *   Waits for `domcontentloaded` and an additional 2-second timeout to ensure JavaScript-rendered content is available.
*   **Content Extraction:** For each page, `trafilatura` is used to extract the main readable text content from the HTML.
*   **Data Structure:** For each successfully crawled page, a dictionary containing `{"url": url, "title": title, "text": text.strip()}` is appended to a list. Only pages with more than 100 characters of extracted text are included.
*   **Output:** The collected data is saved as a JSON array to `backend_RAG/data/book_pages_playwright.json`, formatted with 2-space indentation.

## Execution Requirements
To run the script, the following dependencies must be installed:
```bash
pip install playwright
playwright install chromium
```

## Usage
The script is executed via the Python interpreter:
```bash
python backend_RAG/1_crawl_playwright.py
```

## Expected Outcome
*   Successful crawling of approximately 45 textbook pages.
*   A `backend_RAG/data/book_pages_playwright.json` file containing the extracted data for each page.
*   Upon completion, the message "DONE! {count} pages saved to backend_RAG/data/book_pages_playwright.json" will be printed.
*   The agent should then confirm "Playwright crawl done – continue to Step 2".

## References
*   `sitemap.xml`: `https://physical-ai-textbook-five.vercel.app/sitemap.xml`
*   Output File: `backend_RAG/data/book_pages_playwright.json`
