import os
import tempfile
from typing import Dict, Any
from datetime import datetime

from ...models.textbook import Textbook


class HTMLExporter:
    """
    Service for exporting textbooks to HTML format.
    """

    def __init__(self):
        pass

    def export_to_html(
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
            custom_stylesheet: Path to custom CSS file

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
            "<html lang='en'>",
            "<head>",
            f"  <meta charset='UTF-8'>",
            f"  <meta name='viewport' content='width=device-width, initial-scale=1.0'>",
            f"  <title>{textbook.title}</title>"
        ]

        # Add default or custom stylesheet
        if custom_stylesheet and os.path.exists(custom_stylesheet):
            html_parts.append(f"  <link rel='stylesheet' href='{custom_stylesheet}'>")
        else:
            # Add default responsive styling
            html_parts.append("  <style>")
            html_parts.append("    * { margin: 0; padding: 0; box-sizing: border-box; }")
            html_parts.append("    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; line-height: 1.6; color: #333; background-color: #fff; }")
            html_parts.append("    .container { max-width: 800px; margin: 0 auto; padding: 20px; }")
            html_parts.append("    header { text-align: center; padding: 2rem 0; border-bottom: 2px solid #eee; margin-bottom: 2rem; }")
            html_parts.append("    h1 { color: #2c3e50; font-size: 2.5rem; margin-bottom: 0.5rem; }")
            html_parts.append("    .metadata { color: #7f8c8d; margin-bottom: 1rem; }")
            html_parts.append("    nav { background-color: #f8f9fa; padding: 1rem; border-radius: 5px; margin: 1.5rem 0; }")
            html_parts.append("    nav ul { list-style-type: none; }")
            html_parts.append("    nav li { margin: 0.5rem 0; }")
            html_parts.append("    nav a { text-decoration: none; color: #3498db; font-weight: 500; }")
            html_parts.append("    nav a:hover { text-decoration: underline; }")
            html_parts.append("    .chapter { margin: 3rem 0; padding: 1.5rem 0; border-bottom: 1px solid #eee; }")
            html_parts.append("    .chapter:last-child { border-bottom: none; }")
            html_parts.append("    h2 { color: #34495e; font-size: 1.8rem; margin: 1.5rem 0 1rem 0; padding-bottom: 0.5rem; border-bottom: 1px solid #eee; }")
            html_parts.append("    .section { margin: 1.5rem 0; }")
            html_parts.append("    h3 { color: #2c3e50; font-size: 1.4rem; margin: 1.2rem 0 0.8rem 0; }")
            html_parts.append("    p { margin: 0.8rem 0; }")
            html_parts.append("    .interactive-element { background-color: #f8f9fa; border-left: 4px solid #3498db; padding: 1rem; margin: 1rem 0; border-radius: 0 4px 4px 0; }")
            html_parts.append("    .interactive-element h4 { color: #2980b9; margin-bottom: 0.5rem; }")
            html_parts.append("    footer { text-align: center; padding: 2rem 0; margin-top: 2rem; color: #7f8c8d; border-top: 1px solid #eee; }")
            html_parts.append("    @media (max-width: 768px) {")
            html_parts.append("      .container { padding: 10px; }")
            html_parts.append("      h1 { font-size: 2rem; }")
            html_parts.append("      h2 { font-size: 1.5rem; }")
            html_parts.append("    }")
            html_parts.append("  </style>")

        html_parts.append("</head>")
        html_parts.append("<body>")
        html_parts.append("  <div class='container'>")

        # Add header with title and metadata
        html_parts.append("    <header>")
        html_parts.append(f"      <h1>{textbook.title}</h1>")
        html_parts.append("      <div class='metadata'>")
        html_parts.append(f"        <p><strong>Subject:</strong> {textbook.subject}</p>")
        html_parts.append(f"        <p><strong>Difficulty:</strong> {textbook.difficulty}</p>")
        if textbook.target_audience:
            html_parts.append(f"        <p><strong>Target Audience:</strong> {textbook.target_audience}</p>")
        html_parts.append(f"        <p><strong>Generated:</strong> {datetime.now().strftime('%B %d, %Y')}</p>")
        html_parts.append("      </div>")
        html_parts.append("    </header>")

        # Add navigation if requested
        if include_navigation:
            html_parts.append("    <nav>")
            html_parts.append("      <h3>Table of Contents</h3>")
            html_parts.append("      <ul>")
            for chapter in textbook.chapters:
                html_parts.append(f"        <li><a href='#chapter-{chapter.position}'>Chapter {chapter.position}: {chapter.title}</a></li>")
            html_parts.append("      </ul>")
            html_parts.append("    </nav>")

        # Add chapters and sections
        for chapter in textbook.chapters:
            html_parts.append(f"    <section class='chapter' id='chapter-{chapter.position}'>")
            html_parts.append(f"      <h2>Chapter {chapter.position}: {chapter.title}</h2>")

            for section in chapter.sections:
                html_parts.append(f"      <div class='section'>")
                html_parts.append(f"        <h3>{section.title}</h3>")
                html_parts.append(f"        <p>{section.content}</p>")

                # Add interactive elements if requested
                if include_interactive_elements and section.interactive_elements:
                    for element in section.interactive_elements:
                        html_parts.append(f"        <div class='interactive-element'>")
                        html_parts.append(f"          <h4>{element.type.title()}</h4>")
                        html_parts.append(f"          <p>{element.content}</p>")
                        html_parts.append(f"        </div>")

                html_parts.append(f"      </div>")

            html_parts.append(f"    </section>")

        # Add footer
        html_parts.append("    <footer>")
        html_parts.append(f"      <p>Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>")
        html_parts.append("    </footer>")

        html_parts.append("  </div>")
        html_parts.append("</body>")
        html_parts.append("</html>")

        return "\n".join(html_parts)

    def validate_html_export_options(self, options: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate HTML export options.

        Args:
            options: Dictionary of export options

        Returns:
            Dictionary of validated options with any corrections
        """
        validated_options = {
            'include_navigation': bool(options.get('include_navigation', True)),
            'include_interactive_elements': bool(options.get('include_interactive_elements', True)),
            'include_metadata': bool(options.get('include_metadata', True)),
            'custom_stylesheet': options.get('custom_stylesheet', None),
            'standalone_html': bool(options.get('standalone_html', True))  # Embed CSS or link to external file
        }

        # Validate custom stylesheet path if provided
        if validated_options['custom_stylesheet']:
            if not os.path.exists(validated_options['custom_stylesheet']):
                validated_options['custom_stylesheet'] = None  # Reset if file doesn't exist

        return validated_options