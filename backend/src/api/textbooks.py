from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import List
import asyncio

from ..schemas.textbook_request import TextbookCreateRequest, TextbookStructureRequest
from ..schemas.textbook_response import TextbookResponse, TextbookPreview
from ..schemas.progress_response import ProgressResponse
from ..schemas.export_schemas import ExportRequest
from ..services.textbook_service import TextbookService
from ..services.content_generation.generation_service import ContentGenerationService
from ..exceptions import TextbookNotFoundError, InvalidInputParametersError, TextbookGenerationInProgressError
from ..models.textbook import TextbookStatus

router = APIRouter(prefix="/textbooks", tags=["textbooks"])

# Initialize services
textbook_service = TextbookService()
generation_service = ContentGenerationService()


@router.post("/", response_model=TextbookResponse)
async def create_textbook(request: TextbookCreateRequest):
    """
    Create a new textbook based on provided parameters.
    """
    try:
        # Create the textbook using the service
        textbook = textbook_service.create_textbook(request)

        # Convert to response model
        response = TextbookResponse(
            id=textbook.id,
            title=textbook.title,
            subject=textbook.subject,
            difficulty=textbook.difficulty,
            target_audience=textbook.target_audience,
            chapters=[],
            metadata=textbook.metadata,
            export_formats=textbook.export_formats,
            status=textbook.status,
            created_at=textbook.created_at,
            updated_at=textbook.updated_at
        )

        return response
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to create textbook: {str(e)}")


@router.post("/{textbook_id}/generate")
async def generate_textbook_content(textbook_id: str, background_tasks: BackgroundTasks):
    """
    Start the AI content generation process for a textbook.
    """
    try:
        # Check if generation is already in progress
        if textbook_service.is_generation_active(textbook_id):
            raise TextbookGenerationInProgressError(textbook_id)

        # Start generation process
        started = textbook_service.start_generation(textbook_id)
        if not started:
            raise HTTPException(status_code=409, detail="Could not start generation process")

        # Run generation in the background
        textbook = await generation_service.generate_textbook_content(textbook_id)

        return {
            "id": textbook.id,
            "status": textbook.status,
            "message": "Textbook generation completed successfully"
        }
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {textbook_id} not found")
    except TextbookGenerationInProgressError:
        raise HTTPException(status_code=409, detail=f"Textbook with ID {textbook_id} is already being generated")
    except Exception as e:
        # Update status to failed if generation fails
        try:
            textbook_service.update_textbook_status(textbook_id, TextbookStatus.failed)
        except:
            pass  # Ignore errors when updating status during error handling
        raise HTTPException(status_code=500, detail=f"Failed to generate textbook content: {str(e)}")


@router.post("/{textbook_id}/structure", response_model=TextbookResponse)
async def customize_textbook_structure(textbook_id: str, request: TextbookStructureRequest):
    """
    Customize the structure and format of the generated textbook.
    """
    try:
        # Import here to avoid circular imports
        from ..services.textbook_structure.structure_service import TextbookStructureService

        structure_service = TextbookStructureService()
        textbook = structure_service.customize_textbook_structure(request)

        # Convert to response model
        response = TextbookResponse(
            id=textbook.id,
            title=textbook.title,
            subject=textbook.subject,
            difficulty=textbook.difficulty,
            target_audience=textbook.target_audience,
            chapters=[],  # For now, return empty - would include actual chapters in full implementation
            metadata=textbook.metadata,
            export_formats=textbook.export_formats,
            status=textbook.status,
            created_at=textbook.created_at,
            updated_at=textbook.updated_at
        )

        return response
    except ValidationError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {textbook_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to customize textbook structure: {str(e)}")


@router.post("/{textbook_id}/export")
async def export_textbook(textbook_id: str, request: ExportRequest):
    """
    Export the textbook in the specified format.
    """
    try:
        # Import here to avoid circular imports
        from ..services.export.export_service import ExportService

        export_service = ExportService()
        export_result = export_service.export_textbook(
            textbook_id=textbook_id,
            export_format=request.format,
            include_navigation=request.include_navigation,
            include_interactive_elements=request.include_interactive_elements,
            custom_stylesheet=request.custom_stylesheet
        )

        return export_result
    except ValidationError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except ExportFormatNotSupportedError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {textbook_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to export textbook: {str(e)}")


@router.get("/downloads/{filename}")
async def download_file(filename: str):
    """
    Download an exported textbook file.
    """
    import os
    from fastapi.responses import FileResponse

    # Security: Prevent directory traversal
    filename = os.path.basename(filename)

    # In a real implementation, files would be stored in a secure location
    # and this endpoint would validate access permissions
    file_path = f"docs/textbooks/{filename}"

    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="File not found")

    return FileResponse(
        path=file_path,
        filename=filename,
        media_type='application/octet-stream'
    )


@router.get("/{textbook_id}", response_model=TextbookResponse)
async def get_textbook(textbook_id: str):
    """
    Retrieve details of a specific textbook by ID.
    """
    try:
        textbook = textbook_service.get_textbook(textbook_id)

        # Convert to response model
        response = TextbookResponse(
            id=textbook.id,
            title=textbook.title,
            subject=textbook.subject,
            difficulty=textbook.difficulty,
            target_audience=textbook.target_audience,
            chapters=[],  # For now, return empty - would include actual chapters in full implementation
            metadata=textbook.metadata,
            export_formats=textbook.export_formats,
            status=textbook.status,
            created_at=textbook.created_at,
            updated_at=textbook.updated_at
        )

        return response
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {textbook_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to retrieve textbook: {str(e)}")


@router.get("/{textbook_id}/progress", response_model=ProgressResponse)
async def get_generation_progress(textbook_id: str):
    """
    Check the progress of textbook generation.
    """
    try:
        textbook = textbook_service.get_textbook(textbook_id)

        # Calculate progress (simplified - in real implementation, track actual progress)
        progress_value = 0.0
        if textbook.status == TextbookStatus.draft:
            progress_value = 0.0
        elif textbook.status == TextbookStatus.generating:
            progress_value = 0.5  # Half way when generating
        elif textbook.status == TextbookStatus.complete:
            progress_value = 1.0
        elif textbook.status == TextbookStatus.failed:
            progress_value = 0.0

        response = ProgressResponse(
            id=textbook.id,
            status=textbook.status,
            progress=progress_value,
            message=f"Textbook is {textbook.status.value}"
        )

        return response
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {textbook_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get generation progress: {str(e)}")


@router.get("/{textbook_id}/preview", response_model=TextbookPreview)
async def get_textbook_preview(textbook_id: str):
    """
    Retrieve a preview of the generated textbook content.
    """
    try:
        textbook = textbook_service.get_textbook(textbook_id)

        if textbook.status != TextbookStatus.complete:
            raise HTTPException(
                status_code=409,
                detail=f"Textbook not yet generated. Current status: {textbook.status.value}"
            )

        # Create a preview - in real implementation, this would return actual content
        preview_content = f"Preview of {textbook.title} - {len(textbook.chapters)} chapters available"

        response = TextbookPreview(
            id=textbook.id,
            title=textbook.title,
            preview_content=preview_content,
            chapters_count=len(textbook.chapters)
        )

        return response
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {textbook_id} not found")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get textbook preview: {str(e)}")