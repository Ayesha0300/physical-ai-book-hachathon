# API Contracts: RAG Content Ingestion Pipeline

**Feature**: RAG Content Ingestion Pipeline
**Date**: 2026-01-04
**Branch**: 001-rag-pipeline

## Overview

This document outlines the internal API contracts within the ingestion pipeline, including function signatures and data flow between different components.

## Internal Function Contracts

### 1. fetch_sitemap_urls()
**Purpose**: Fetch and parse the sitemap.xml to extract all URLs

**Input**:
- sitemap_url (string): URL of the sitemap.xml file

**Output**:
- List[string]: List of URLs extracted from the sitemap
- Raises: Exception if sitemap cannot be fetched or parsed

**Contract**:
- Must handle HTTP errors gracefully
- Must parse standard sitemap.xml format
- Must return only valid URLs

### 2. fetch_page_content(url)
**Purpose**: Fetch the HTML content of a single page

**Input**:
- url (string): URL of the page to fetch

**Output**:
- dict: { 'url': string, 'title': string, 'content': string, 'headings': list }

**Contract**:
- Must handle network timeouts
- Must extract title from HTML
- Must extract main content from HTML body
- Must extract heading hierarchy
- Must return original URL with content

### 3. chunk_content(extracted_page, chunk_size, chunk_overlap)
**Purpose**: Split extracted page content into smaller chunks

**Input**:
- extracted_page (dict): Page data from fetch_page_content
- chunk_size (int): Maximum size of each chunk
- chunk_overlap (int): Overlap between chunks

**Output**:
- List[dict]: List of content chunks with metadata

**Contract**:
- Must preserve source URL in each chunk
- Must maintain chunk ordering information
- Must not break semantic meaning within chunks
- Must include relevant metadata in each chunk

### 4. generate_embeddings(content_chunks)
**Purpose**: Generate vector embeddings for content chunks using Cohere

**Input**:
- content_chunks (List[dict]): List of content chunks to embed

**Output**:
- List[dict]: Content chunks with added embedding vectors

**Contract**:
- Must handle API rate limits gracefully
- Must preserve all original chunk data
- Must add embedding vectors of consistent dimension
- Must handle API errors and retry appropriately

### 5. store_vectors(embedded_chunks)
**Purpose**: Store embedded chunks in Qdrant vector database

**Input**:
- embedded_chunks (List[dict]): Content chunks with embeddings

**Output**:
- dict: { 'success_count': int, 'error_count': int, 'errors': list }

**Contract**:
- Must store all valid embeddings
- Must return counts of successful and failed operations
- Must preserve all metadata in payload
- Must handle database connection issues

### 6. main()
**Purpose**: Orchestrate the entire ingestion workflow

**Input**:
- None (reads configuration from environment variables)

**Output**:
- None (writes results to vector database and logs)

**Contract**:
- Must execute all pipeline steps sequentially
- Must handle errors gracefully at each stage
- Must provide progress updates
- Must validate final results