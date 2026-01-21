import requests
import time

# URLs found in the HTML of the main page
html_links = [
    "https://physical-ai-book-hachathon.vercel.app/modules",
    "https://physical-ai-book-hachathon.vercel.app/docs/capstone",
    "https://physical-ai-book-hachathon.vercel.app/docs/hardware",
    "https://physical-ai-book-hachathon.vercel.app/docs/intro",
    "https://physical-ai-book-hachathon.vercel.app/docs/module1-ros2/chapter1-why-robots-need-nervous-system",
    "https://physical-ai-book-hachathon.vercel.app/docs/module2-digital-twin/chapter1-intro",
    "https://physical-ai-book-hachathon.vercel.app/docs/module3-isaac/chapter1-from-middleware-to-intelligence",
    "https://physical-ai-book-hachathon.vercel.app/docs/vla/chapter-1-why-vla-matters",
    "https://physical-ai-book-hachathon.vercel.app/docs/vla/vla-fundamentals",
    "https://physical-ai-book-hachathon.vercel.app/docs/glossary",
    "https://physical-ai-book-hachathon.vercel.app/docs/quickstart"
]

accessible_urls = []

for url in html_links:
    print(f"Testing {url}...", end=" ")
    try:
        response = requests.get(url, timeout=10)
        if response.status_code == 200:
            print("OK")
            accessible_urls.append(url)
        else:
            print(f"{response.status_code}")
    except Exception as e:
        print(f"Error: {e}")

    time.sleep(0.1)  # Be respectful to the server

print(f"\nFound {len(accessible_urls)} accessible URLs:")
for url in accessible_urls:
    print(f"  - {url}")

# Also write to file
with open('html_discovered_urls.txt', 'w') as f:
    for url in accessible_urls:
        f.write(url + '\n')

print("\nSaved to html_discovered_urls.txt")