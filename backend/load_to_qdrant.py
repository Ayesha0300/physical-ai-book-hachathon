import requests
from bs4 import BeautifulSoup
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import uuid
import textwrap

# Qdrant setup
qdrant_client = QdrantClient("http://localhost:6333")
collection_name = "book_docs"  # Match your COLLECTION_NAME

# Embedding model (matches MCP default, dimension 384)
embedder = SentenceTransformer("sentence-transformers/all-MiniLM-L6-v2")

# Fetch and parse sitemap to get URLs
sitemap_url = "https://physical-ai-book-hachathon.vercel.app/sitemap.xml"
response = requests.get(sitemap_url)
soup = BeautifulSoup(response.text, "xml")
urls = [loc.text for loc in soup.find_all("loc")]
print(f"Found {len(urls)} URLs in sitemap.")

# Create collection if it doesn't exist (vector size matches embedding model)
if not qdrant_client.has_collection(collection_name):
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=384, distance=Distance.COSINE),
    )

# Function to scrape text from a URL
def scrape_text(url):
    try:
        response = requests.get(url)
        soup = BeautifulSoup(response.text, "html.parser")
        # Extract main text (adjust selectors if needed for your site's structure)
        text = " ".join(p.text for p in soup.find_all("p"))
        return text.strip()
    except Exception as e:
        print(f"Error scraping {url}: {e}")
        return ""

# Process each URL
points = []
for url in urls:
    content = scrape_text(url)
    if not content:
        continue
    
    # Chunk text into smaller pieces (e.g., 500 chars) for better embeddings
    chunks = textwrap.wrap(content, width=500, break_long_words=False)
    
    for chunk in chunks:
        embedding = embedder.encode(chunk).tolist()
        point_id = str(uuid.uuid4())  # Unique ID
        points.append(PointStruct(
            id=point_id,
            vector=embedding,
            payload={"url": url, "text": chunk}
        ))

# Upsert points in batches (to avoid overload)
batch_size = 50
for i in range(0, len(points), batch_size):
    qdrant_client.upsert(
        collection_name=collection_name,
        points=points[i:i + batch_size]
    )
    print(f"Upserted batch {i // batch_size + 1}")

print("Data loaded into Qdrant. Check dashboard at http://localhost:6333/dashboard")