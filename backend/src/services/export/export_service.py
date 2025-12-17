import os
import tempfile
from typing import Dict, Any
from datetime import datetime, timedelta
import uuid

from ...models.textbook import Textbook
from ...services.textbook_service import TextbookService
from ...utils.storage import FileStorage
from ...exceptions import TextbookNotFoundError, ExportFormatNotSupportedError, ValidationError
from ..base_service import BaseService
from ..content_generation.interactive_service import InteractiveElementService


class ExportService(BaseService):
    """
    Service for exporting textbooks in various formats.
    """

    def __init__(self):
        super().__init__()
        self.textbook_service = TextbookService()
        self.storage = FileStorage()
        self.interactive_service = InteractiveElementService()

        # Supported export formats
        self.supported_formats = {
            'pdf': {'extension': '.pdf', 'handler': self._export_to_pdf},
            'html': {'extension': '.html', 'handler': self._export_to_html},
            'markdown': {'extension': '.md', 'handler': self._export_to_markdown}
        }

    def export_textbook(
        self,
        textbook_id: str,
        export_format: str,
        include_navigation: bool = True,
        include_interactive_elements: bool = True,
        custom_stylesheet: str = None
    ) -> Dict[str, Any]:
        """
        Export a textbook in the specified format.

        Args:
            textbook_id: ID of the textbook to export
            export_format: Format to export to (pdf, html, markdown)
            include_navigation: Whether to include navigation elements
            include_interactive_elements: Whether to include interactive elements
            custom_stylesheet: Path to custom CSS for HTML/PDF export

        Returns:
            Dictionary with export information
        """
        # Validate export format
        if export_format.lower() not in self.supported_formats:
            raise ExportFormatNotSupportedError(export_format)

        # Get the textbook
        textbook = self.textbook_service.get_textbook(textbook_id)

        # Check if textbook is ready for export
        if textbook.status != 'complete':
            raise ValidationError(f"Textbook must be in 'complete' status for export, current status: {textbook.status}")

        # Get the handler for the requested format
        format_info = self.supported_formats[export_format.lower()]
        handler = format_info['handler']

        # Export the textbook
        file_path = handler(
            textbook,
            include_navigation=include_navigation,
            include_interactive_elements=include_interactive_elements,
            custom_stylesheet=custom_stylesheet
        )

        # Generate download URL (in a real implementation, this would be a temporary URL)
        download_url = f"/downloads/{os.path.basename(file_path)}"

        # Calculate file size
        file_size = os.path.getsize(file_path) if os.path.exists(file_path) else 0

        # Set expiration time (24 hours from now)
        expires_at = datetime.now() + timedelta(hours=24)

        return {
            "download_url": download_url,
            "format": export_format.lower(),
            "file_size": file_size,
            "expires_at": expires_at,
            "message": f"Textbook exported successfully as {export_format.upper()}"
        }

    def _export_to_pdf(
        self,
        textbook: Textbook,
        include_navigation: bool = True,
        include_interactive_elements: bool = True,
        custom_stylesheet: str = None
    ) -> str:
        """
        Export textbook to PDF format.

        Args:
            textbook: Textbook to export
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements
            custom_stylesheet: Path to custom CSS

        Returns:
            Path to the exported PDF file
        """
        # For now, create a placeholder PDF content
        # In a real implementation, this would use a PDF generation library like reportlab
        content = self._generate_export_content(
            textbook,
            include_navigation,
            include_interactive_elements,
            format_type="pdf"
        )

        # Create a temporary file
        temp_file = tempfile.NamedTemporaryFile(
            delete=False,
            suffix='.pdf',
            prefix=f'textbook_{textbook.id}_'
        )
        temp_file.write(content.encode('utf-8'))
        temp_file.close()

        # In a real implementation, this would generate an actual PDF
        # For now, we'll create a text file with PDF extension to simulate
        pdf_path = temp_file.name.replace('.pdf', '_simulated.pdf')
        with open(pdf_path, 'w', encoding='utf-8') as f:
            f.write(f"PDF Export of {textbook.title}\n")
            f.write("=" * 40 + "\n")
            f.write(content)

        return pdf_path

    def _export_to_html(
        self,
        textbook: Textbook,
        include_navigation: bool = True,
        include_interactive_elements: bool = True,
        custom_stylesheet: str = None
    ) -> str:
        """
        Export textbook to HTML format.

        Args:
            textbook: Textbook to export
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements
            custom_stylesheet: Path to custom CSS

        Returns:
            Path to the exported HTML file
        """
        # Generate HTML content
        html_content = self._generate_html_content(
            textbook,
            include_navigation,
            include_interactive_elements,
            custom_stylesheet
        )

        # Create a temporary file
        temp_file = tempfile.NamedTemporaryFile(
            delete=False,
            suffix='.html',
            prefix=f'textbook_{textbook.id}_'
        )
        temp_file.write(html_content.encode('utf-8'))
        temp_file.close()

        return temp_file.name

    def _export_to_markdown(
        self,
        textbook: Textbook,
        include_navigation: bool = True,
        include_interactive_elements: bool = True,
        custom_stylesheet: str = None
    ) -> str:
        """
        Export textbook to Markdown format.

        Args:
            textbook: Textbook to export
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements
            custom_stylesheet: Path to custom CSS (ignored for markdown)

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

    def _generate_export_content(
        self,
        textbook: Textbook,
        include_navigation: bool,
        include_interactive_elements: bool,
        format_type: str = "text"
    ) -> str:
        """
        Generate export content in the specified format.

        Args:
            textbook: Textbook to generate content for
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements
            format_type: Type of format to generate for

        Returns:
            Generated content string
        """
        content_parts = []

        # Add title
        content_parts.append(f"Title: {textbook.title}")
        content_parts.append(f"Subject: {textbook.subject}")
        content_parts.append(f"Difficulty: {textbook.difficulty}")
        content_parts.append("")

        # Add chapters
        for chapter in textbook.chapters:
            content_parts.append(f"Chapter {chapter.position}: {chapter.title}")
            content_parts.append("-" * 40)

            # Add sections
            for section in chapter.sections:
                content_parts.append(f"Section: {section.title}")
                content_parts.append(section.content)
                content_parts.append("")

                # Add interactive elements if requested
                if include_interactive_elements and section.interactive_elements:
                    for element in section.interactive_elements:
                        content_parts.append(f"[{element.type.upper()}]: {element.content}")
                        content_parts.append("")

            content_parts.append("=" * 50)  # Separator between chapters
            content_parts.append("")

        return "\n".join(content_parts)

    def _generate_html_content(
        self,
        textbook: Textbook,
        include_navigation: bool,
        include_interactive_elements: bool,
        custom_stylesheet: str = None
    ) -> str:
        """
        Generate HTML content for the textbook.

        Args:
            textbook: Textbook to generate HTML for
            include_navigation: Whether to include navigation
            include_interactive_elements: Whether to include interactive elements
            custom_stylesheet: Path to custom CSS

        Returns:
            HTML content string
        """
        # Start building HTML
        html_parts = [
            "<!DOCTYPE html>",
            "<html>",
            "<head>",
            f"  <title>{textbook.title}</title>",
            "  <meta charset='UTF-8'>",
            "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        ]

        # Add default or custom stylesheet
        if custom_stylesheet and os.path.exists(custom_stylesheet):
            html_parts.append(f"  <link rel='stylesheet' href='{custom_stylesheet}'>")
        else:
            html_parts.append("  <style>")
            html_parts.append("    body { font-family: Arial, sans-serif; line-height: 1.6; margin: 40px; }")
            html_parts.append("    h1, h2, h3 { color: #333; }")
            html_parts.append("    .chapter { margin-bottom: 40px; }")
            html_parts.append("    .section { margin-bottom: 20px; }")
            html_parts.append("    .interactive-element { background-color: #f0f8ff; padding: 10px; margin: 10px 0; border-left: 3px solid #007acc; }")
            html_parts.append("  </style>")

        html_parts.append("</head>")
        html_parts.append("<body>")

        # Add title and metadata
        html_parts.append(f"  <h1>{textbook.title}</h1>")
        html_parts.append(f"  <p><strong>Subject:</strong> {textbook.subject}</p>")
        html_parts.append(f"  <p><strong>Difficulty:</strong> {textbook.difficulty}</p>")

        # Add navigation if requested
        if include_navigation:
            html_parts.append("  <nav>")
            html_parts.append("    <h2>Table of Contents</h2>")
            html_parts.append("    <ul>")
            for chapter in textbook.chapters:
                html_parts.append(f"      <li><a href='#chapter-{chapter.position}'>Chapter {chapter.position}: {chapter.title}</a></li>")
            html_parts.append("    </ul>")
            html_parts.append("  </nav>")

        # Add chapters and sections
        for chapter in textbook.chapters:
            html_parts.append(f"  <div class='chapter' id='chapter-{chapter.position}'>")
            html_parts.append(f"    <h2>Chapter {chapter.position}: {chapter.title}</h2>")

            for section in chapter.sections:
                html_parts.append(f"    <div class='section'>")
                html_parts.append(f"      <h3>{section.title}</h3>")
                html_parts.append(f"      <p>{section.content}</p>")

                # Add interactive elements if requested
                if include_interactive_elements and section.interactive_elements:
                    for element in section.interactive_elements:
                        html_parts.append(f"      <div class='interactive-element'>")
                        html_parts.append(f"        <h4>{element.type.title()}:</h4>")
                        html_parts.append(f"        <p>{element.content}</p>")
                        html_parts.append(f"      </div>")

                html_parts.append(f"    </div>")

            html_parts.append(f"  </div>")

        html_parts.append("</body>")
        html_parts.append("</html>")

        return "\n".join(html_parts)

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

        # Add title and metadata
        md_parts.append(f"# {textbook.title}")
        md_parts.append(f"**Subject:** {textbook.subject}")
        md_parts.append(f"**Difficulty:** {textbook.difficulty}")
        md_parts.append("")

        # Add navigation if requested
        if include_navigation:
            md_parts.append("## Table of Contents")
            for chapter in textbook.chapters:
                md_parts.append(f"- [Chapter {chapter.position}: {chapter.title}](#chapter-{chapter.position})")
            md_parts.append("")

        # Add chapters and sections
        for chapter in textbook.chapters:
            md_parts.append(f"## Chapter {chapter.position}: {chapter.title} {{#chapter-{chapter.position}}}")
            md_parts.append("")

            for section in chapter.sections:
                md_parts.append(f"### {section.title}")
                md_parts.append(section.content)
                md_parts.append("")

                # Add interactive elements if requested
                if include_interactive_elements and section.interactive_elements:
                    for element in section.interactive_elements:
                        md_parts.append(f"**{element.type.upper()}:**")
                        md_parts.append(element.content)
                        md_parts.append("")

            md_parts.append("---")  # Separator between chapters
            md_parts.append("")

        return "\n".join(md_parts)

    def get_export_options(self, textbook: Textbook) -> Dict[str, Any]:
        """
        Get available export options for a textbook.

        Args:
            textbook: Textbook to get export options for

        Returns:
            Dictionary with available export options
        """
        return {
            "supported_formats": list(self.supported_formats.keys()),
            "default_format": "pdf",
            "options": {
                "include_navigation": True,
                "include_interactive_elements": True,
                "custom_stylesheet": False
            },
            "format_descriptions": {
                "pdf": "Portable Document Format - Good for printing and sharing",
                "html": "HyperText Markup Language - Good for web viewing",
                "markdown": "Lightweight markup - Good for documentation systems"
            }
        }