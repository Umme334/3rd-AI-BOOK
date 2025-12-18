#!/usr/bin/env python3
"""
Script to convert HTML files to Markdown files in the textbooks directory.
"""
import os
import html2text
from pathlib import Path

def convert_html_to_md(html_file_path):
    """Convert a single HTML file to Markdown."""
    with open(html_file_path, 'r', encoding='utf-8') as html_file:
        html_content = html_file.read()

    # Configure html2text
    h = html2text.HTML2Text()
    h.ignore_links = False
    h.ignore_images = False
    h.body_width = 0  # Don't wrap lines
    h.single_line_break = True

    # Convert HTML to Markdown
    md_content = h.handle(html_content)

    # Create the markdown file path by replacing .html with .md
    md_file_path = html_file_path.with_suffix('.md')

    # Write the markdown content
    with open(md_file_path, 'w', encoding='utf-8') as md_file:
        md_file.write(md_content)

    print(f"Converted: {html_file_path} -> {md_file_path}")
    return md_file_path

def main():
    """Main function to find and convert HTML files to Markdown."""
    # Find all HTML files in the textbooks directory
    textbooks_dir = Path("textbooks")

    if not textbooks_dir.exists():
        print(f"Directory {textbooks_dir} does not exist.")
        return

    # Find all HTML files
    html_files = list(textbooks_dir.rglob("*.html"))

    print(f"Found {len(html_files)} HTML files to convert.")

    converted_count = 0
    for html_file in html_files:
        try:
            # Skip files in node_modules or build directories
            if "node_modules" in str(html_file) or "build" in str(html_file):
                continue

            convert_html_to_md(html_file)
            converted_count += 1
        except Exception as e:
            print(f"Error converting {html_file}: {str(e)}")

    print(f"\nConversion completed! Converted {converted_count} HTML files to Markdown.")

if __name__ == "__main__":
    main()