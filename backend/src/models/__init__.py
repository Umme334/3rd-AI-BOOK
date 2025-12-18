"""
Module to handle model rebuilding for circular references in Pydantic models.
This resolves the forward reference issues in the textbook models.
"""
# Import models in the right order to ensure dependencies are available
def rebuild_models():
    """Rebuild all models to resolve forward references."""
    # Import models in a way that avoids circular dependencies during import
    from .section import Section
    from .chapter import Chapter
    from .textbook import Textbook

    # Rebuild models in the correct order to resolve forward references
    # First rebuild the innermost models
    Section.model_rebuild()
    Chapter.model_rebuild()
    Textbook.model_rebuild()
    print("Models rebuilt successfully to resolve forward references")

# Call the rebuild function to ensure models are properly set up
rebuild_models()