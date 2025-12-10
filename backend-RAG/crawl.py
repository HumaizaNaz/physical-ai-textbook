import requests
from bs4 import BeautifulSoup
import trafilatura
import json
import os

def crawl_sitemap(session, sitemap_url):
    session = requests.Session()
    session.headers.update({
        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36',
        'Accept-Encoding': 'gzip, deflate, br',
        'Connection': 'keep-alive',
        'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9',
        'Accept-Language': 'en-US,en;q=0.9',
    })
    
    response = session.get(sitemap_url)
    response.raise_for_status()
    soup = BeautifulSoup(response.content, 'xml')
    urls = [loc.text for loc in soup.find_all('loc')]
    return urls

def extract_text_from_url(session, url):
    try:
        response = session.get(url, timeout=10)
        response.raise_for_status()
        text = trafilatura.extract(response.text, favor_recall=True, include_comments=False, include_images=False, include_links=False)
        return text
    except requests.exceptions.RequestException as e:
        print(f"Error fetching {url}: {e}")
        return None

def main():
    sitemap_url = "https://physical-ai-textbook-five.vercel.app/sitemap.xml"
    output_dir = "backend-RAG/data"
    output_file = os.path.join(output_dir, "book_texts.json")

    session = requests.Session()
    session.headers.update({
        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36',
        'Accept-Encoding': 'gzip, deflate, br',
        'Connection': 'keep-alive',
        'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9',
        'Accept-Language': 'en-US,en;q=0.9',
    })

    print(f"Crawling sitemap: {sitemap_url}")
    urls = crawl_sitemap(session, sitemap_url)
    print(f"Found {len(urls)} URLs.")

    book_texts = {}
    for i, url in enumerate(urls):
        print(f"Processing URL {i+1}/{len(urls)}: {url}")
        text = extract_text_from_url(session, url)
        if text:
            book_texts[url] = text
        else:
            print(f"Could not extract text from {url}")

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(book_texts, f, ensure_ascii=False, indent=4)
    print(f"Extracted texts saved to {output_file}")

if __name__ == "__main__":
    main()
