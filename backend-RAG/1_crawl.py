import json
import os
import time
from bs4 import BeautifulSoup
from playwright.sync_api import sync_playwright
import trafilatura

def crawl_sitemap(browser, sitemap_url):
    try:
        page = browser.new_page()
        page.goto(sitemap_url)
        content = page.content()
        page.close()

        soup = BeautifulSoup(content, 'xml')
        urls = [loc.text for loc in soup.find_all('loc')]
        return urls
    except Exception as e:
        print(f"Error fetching sitemap {sitemap_url}: {e}")
        return []

def extract_text_and_title_from_url(browser, url):
    try:
        page = browser.new_page()
        page.goto(url)
        
        title = page.title()
        content = page.content()
        page.close()

        text = trafilatura.extract(content, favor_recall=True, include_comments=False, include_images=False, include_links=False)
        
        if text:
            return title, text
        return None, None
    except Exception as e:
        print(f"Error fetching {url}: {e}")
        return None, None

def main():
    sitemap_url = "https://physical-ai-textbook-five.vercel.app/sitemap.xml"
    output_dir = "backend-RAG/data"
    output_file = os.path.join(output_dir, "book_pages.json")

    with sync_playwright() as p:
        browser = p.chromium.launch() # Launch browser once
        print(f"Crawling sitemap: {sitemap_url}")
        urls = crawl_sitemap(browser, sitemap_url)
        print(f"Found {len(urls)} URLs.")

        crawled_data = []
        for i, url in enumerate(urls):
            print(f"Processing URL {i+1}/{len(urls)}: {url}")
            title, text = extract_text_and_title_from_url(browser, url)
            if title and text:
                crawled_data.append({"url": url, "title": title, "text": text})
            else:
                print(f"Could not extract title or text from {url}")
            time.sleep(1) # Be polite and avoid hammering the server
        
        browser.close() # Close browser at the end

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(crawled_data, f, ensure_ascii=False, indent=4)
    
    print(f"STEP 1 DONE – Book crawled. Ready for Step 2 → bolo Continue")

if __name__ == "__main__":
    main()