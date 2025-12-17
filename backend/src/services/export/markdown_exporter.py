import os
import tempfile
from typing import Dict, Any
from datetime import datetime

from ...models.textbook import Textbook


class MarkdownExporter:
    """
    Service for exporting textbooks to Markdown format.
    """

    def __init__(self):
        pass

    def export_to_markdown(
        self,
        textbook: Textbook,
        include_navigation: bool = True,
        include_interactive_elements: bool = True
    ) -> str:
        """
        Export textbook to Markdown format.

        Args:
            textbook: Textbook to export
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements

        Returns:
            Path to the exported Markdown file
        """
        # Generate Markdown content
        md_content = self._generate_markdown_content(
            textbook,
            include_navigation,
            include_interactive_elements
        )

        # Create a temporary file
        temp_file = tempfile.NamedTemporaryFile(
            delete=False,
            suffix='.md',
            prefix=f'textbook_{textbook.id}_'
        )
        temp_file.write(md_content.encode('utf-8'))
        temp_file.close()

        return temp_file.name

    def _generate_markdown_content(
        self,
        textbook: Textbook,
        include_navigation: bool,
        include_interactive_elements: bool
    ) -> str:
        """
        Generate Markdown content for the textbook.

        Args:
            textbook: Textbook to generate Markdown for
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements

        Returns:
            Markdown content string
        """
        md_parts = []

        # Add front matter (YAML header for documentation systems)
        md_parts.append("---")
        md_parts.append(f"title: {textbook.title}")
        md_parts.append(f"subject: {textbook.subject}")
        md_parts.append(f"difficulty: {textbook.difficulty}")
        if textbook.target_audience:
            md_parts.append(f"target_audience: {textbook.target_audience}")
        md_parts.append(f"generated: {datetime.now().isoformat()}")
        md_parts.append("---")
        md_parts.append("")  # Empty line after front matter

        # Add title and metadata
        md_parts.append(f"# {textbook.title}")
        md_parts.append("")
        md_parts.append(f"**Subject:** {textbook.subject}")
        md_parts.append(f"**Difficulty:** {textbook.difficulty}")
        if textbook.target_audience:
            md_parts.append(f"**Target Audience:** {textbook.target_audience}")
        md_parts.append(f"**Generated:** {datetime.now().strftime('%B %d, %Y')}")
        md_parts.append("")

        # Add navigation if requested
        if include_navigation:
            md_parts.append("## Table of Contents")
            for chapter in textbook.chapters:
                md_parts.append(f"- [Chapter {chapter.position}: {chapter.title}](#chapter-{chapter.position.replace(' ', '-').lower()})")
            md_parts.append("")

        # Add chapters and sections
        for chapter in textbook.chapters:
            # Create anchor-friendly chapter ID
            chapter_id = f"chapter-{chapter.position}-{chapter.title.replace(' ', '-').replace(':', '').lower()}"

            md_parts.append(f"## Chapter {chapter.position}: {chapter.title} {{#{chapter_id}}}")
            md_parts.append("")

            for section in chapter.sections:
                md_parts.append(f"### {section.title}")
                md_parts.append("")
                md_parts.append(section.content)
                md_parts.append("")

                # Add interactive elements if requested
                if include_interactive_elements and section.interactive_elements:
                    for element in section.interactive_elements:
                        md_parts.append(f"**{element.type.upper()}:**")
                        md_parts.append("")
                        md_parts.append(element.content)
                        md_parts.append("")

            # Add horizontal rule between chapters
            md_parts.append("---")
            md_parts.append("")

        # Add metadata at the end
        md_parts.append("")
        md_parts.append("## Document Information")
        md_parts.append(f"- **Exported:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        md_parts.append(f"- **Format:** Markdown")
        md_parts.append(f"- **Source:** Textbook Generation System")

        return "\n".join(md_parts)

    def validate_markdown_export_options(self, options: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate Markdown export options.

        Args:
            options: Dictionary of export options

        Returns:
            Dictionary of validated options with any corrections
        """
        validated_options = {
            'include_navigation': bool(options.get('include_navigation', True)),
            'include_interactive_elements': bool(options.get('include_interactive_elements', True)),
            'include_front_matter': bool(options.get('include_front_matter', True)),  # YAML front matter
            'use_anchors': bool(options.get('use_anchors', True)),  # Add anchors for headings
            'format_type': options.get('format_type', 'standard').lower()  # 'standard', 'docusaurus', 'jekyll'
        }

        # Validate format type
        valid_formats = ['standard', 'docusaurus', 'jekyll', 'github']
        if validated_options['format_type'] not in valid_formats:
            validated_options['format_type'] = 'standard'

        return validated_options

    def export_to_docusaurus_format(
        self,
        textbook: Textbook,
        include_navigation: bool = True,
        include_interactive_elements: bool = True
    ) -> str:
        """
        Export textbook to Docusaurus-compatible Markdown format.

        Args:
            textbook: Textbook to export
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements

        Returns:
            Path to the exported Docusaurus Markdown file
        """
        # Generate Docusaurus-specific Markdown content
        md_content = self._generate_docusaurus_content(
            textbook,
            include_navigation,
            include_interactive_elements
        )

        # Create a temporary file with proper naming for Docusaurus
        # Docusaurus typically uses numbered prefixes for ordering
        temp_file = tempfile.NamedTemporaryFile(
            delete=False,
            suffix='.md',
            prefix=f'textbook_{textbook.id}_docusaurus_'
        )
        temp_file.write(md_content.encode('utf-8'))
        temp_file.close()

        return temp_file.name

    def _generate_docusaurus_content(
        self,
        textbook: Textbook,
        include_navigation: bool,
        include_interactive_elements: bool
    ) -> str:
        """
        Generate Docusaurus-compatible Markdown content.

        Args:
            textbook: Textbook to generate content for
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements

        Returns:
            Docusaurus-compatible Markdown content string
        """
        md_parts = []

        # Add Docusaurus-specific front matter
        md_parts.append("---")
        md_parts.append(f"id: {textbook.id}")
        md_parts.append(f"title: {textbook.title}")
        md_parts.append(f"sidebar_label: {textbook.title}")
        md_parts.append("description: " + textbook.title)
        md_parts.append(f"keywords:")
        md_parts.append(f"  - {textbook.subject}")
        md_parts.append(f"  - textbook")
        md_parts.append(f"  - education")
        md_parts.append("---")
        md_parts.append("")

        # Add title
        md_parts.append(f"# {textbook.title}")
        md_parts.append("")
        md_parts.append(f"**Subject:** {textbook.subject}")
        md_parts.append(f"**Difficulty:** {textbook.difficulty}")
        if textbook.target_audience:
            md_parts.append(f"**Target Audience:** {textbook.target_audience}")
        md_parts.append("")

        # Add chapters and sections
        for chapter in textbook.chapters:
            md_parts.append(f"## Chapter {chapter.position}: {chapter.title}")
            md_parts.append("")

            for section in chapter.sections:
                md_parts.append(f"### {section.title}")
                md_parts.append("")
                md_parts.append(section.content)
                md_parts.append("")

                # Add interactive elements if requested
                if include_interactive_elements and section.interactive_elements:
                    for element in section.interactive_elements:
                        md_parts.append(f"**{element.type.upper()}:**")
                        md_parts.append("")
                        md_parts.append(element.content)
                        md_parts.append("")

            # Add horizontal rule between chapters
            md_parts.append("---")
            md_parts.append("")

        return "\n".join(md_parts)