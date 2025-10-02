"""
Collage Maker - Quick 2x2 photo collage builder for Windows.

README
======
Installation::
    py -m venv venv
    venv\Scripts\activate
    pip install -r requirements.txt

Run::
    python collage_maker.py

Packaging hint::
    pyinstaller --noconsole --onefile collage_maker.py
"""
from __future__ import annotations

import dataclasses
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageOps

from PySide6.QtCore import (QByteArray, QObject, QPoint, QPointF, QRectF, QRunnable,
                            QSize, QSizeF, Qt, QThreadPool, QTimer, Signal, QMimeData)
from PySide6.QtGui import (QAction, QColor, QContextMenuEvent, QDrag, QDragEnterEvent,
                           QDragMoveEvent, QDropEvent, QIcon, QImage, QKeySequence,
                            QMouseEvent, QPainter, QPen, QPixmap,
                           QResizeEvent, QWheelEvent)
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QCheckBox, QComboBox,
                               QDialog, QDialogButtonBox, QFileDialog, QFormLayout,
                               QGraphicsItem, QGraphicsObject, QGraphicsScene,
                               QGraphicsSceneHoverEvent, QGraphicsView, QHBoxLayout,
                               QLabel, QListWidget, QListWidgetItem, QMainWindow, QMenu,
                               QMessageBox, QPushButton, QSlider, QSpinBox, QSplitter,
                               QStyle, QStyleOptionGraphicsItem, QToolBar, QVBoxLayout,
                               QWidget)

APP_NAME = "CollageMaker"
CONFIG_FILENAME = "settings.json"
SUPPORTED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".heic", ".heif"}
THUMBNAIL_SIZE = QSize(128, 128)
PREVIEW_LONG_EDGE = 2048
DEFAULT_LONG_SIDE = 6000
DEFAULT_DPI = 300
DEFAULT_QUALITY = 95
UNDO_HISTORY_LIMIT = 20
DEFAULT_BORDER_WIDTH = 12


@dataclass
class CollageSettings:
    long_side: int = DEFAULT_LONG_SIDE
    image_format: str = "JPEG"
    jpeg_quality: int = DEFAULT_QUALITY
    dpi: int = DEFAULT_DPI
    remove_used: bool = True
    border_width: int = DEFAULT_BORDER_WIDTH

    def to_dict(self) -> Dict[str, object]:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "CollageSettings":
        kwargs = {}
        for field_name in ("long_side", "image_format", "jpeg_quality", "dpi", "remove_used"):
            if field_name in data:
                kwargs[field_name] = data[field_name]
        if "border_width" in data:
            kwargs["border_width"] = data["border_width"]
        return cls(**kwargs)


class ConfigManager:
    def __init__(self) -> None:
        self.config_dir = self._resolve_config_dir()
        self.config_path = self.config_dir / CONFIG_FILENAME
        self.settings = CollageSettings()
        self.last_open_dir: Optional[str] = None
        self.last_save_dir: Optional[str] = None
        self._load()

    def _resolve_config_dir(self) -> Path:
        from PySide6.QtCore import QStandardPaths

        location = QStandardPaths.writableLocation(QStandardPaths.AppConfigLocation)
        if not location:
            location = os.path.join(Path.home(), f".{APP_NAME.lower()}")
        config_dir = Path(location)
        config_dir.mkdir(parents=True, exist_ok=True)
        return config_dir

    def _load(self) -> None:
        if not self.config_path.exists():
            return
        try:
            data = json.loads(self.config_path.read_text(encoding="utf-8"))
            settings = data.get("settings", {})
            self.settings = CollageSettings.from_dict(settings)
            self.last_open_dir = data.get("last_open_dir")
            self.last_save_dir = data.get("last_save_dir")
        except Exception:
            # Ignore config corruption; start fresh.
            pass

    def save(self) -> None:
        payload = {
            "settings": self.settings.to_dict(),
            "last_open_dir": self.last_open_dir,
            "last_save_dir": self.last_save_dir,
        }
        self.config_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


@dataclass
class PhotoEntry:
    path: str
    pil_image: Image.Image
    preview_qimage: QImage
    thumbnail: QPixmap

    def clone_for_cell(self) -> "PhotoEntry":
        return PhotoEntry(
            path=self.path,
            pil_image=self.pil_image,
            preview_qimage=self.preview_qimage,
            thumbnail=self.thumbnail,
        )


class WorkerSignals(QObject):
    finished = Signal(str, object)
    failed = Signal(str, str)


class ThumbnailWorker(QRunnable):
    def __init__(self, path: str, signals: WorkerSignals):
        super().__init__()
        self.path = path
        self.signals = signals

    def run(self) -> None:
        try:
            entry = self._load_photo(self.path)
            self.signals.finished.emit(self.path, entry)
        except Exception as exc:
            self.signals.failed.emit(self.path, str(exc))

    @staticmethod
    def _load_photo(path: str) -> PhotoEntry:
        path_obj = Path(path)
        if not path_obj.exists():
            raise FileNotFoundError(path)

        with Image.open(path_obj) as img:
            img = ImageOps.exif_transpose(img)
            pil_image = img.convert("RGB")

        preview = pil_image.copy()
        preview.thumbnail((PREVIEW_LONG_EDGE, PREVIEW_LONG_EDGE), Image.Resampling.LANCZOS)
        preview_data = preview.tobytes("raw", "RGB")
        preview_qimage = QImage(preview_data, preview.width, preview.height, preview.width * 3, QImage.Format_RGB888)
        preview_qimage = preview_qimage.copy()

        thumb = preview.copy()
        thumb.thumbnail((THUMBNAIL_SIZE.width(), THUMBNAIL_SIZE.height()), Image.Resampling.LANCZOS)
        thumb_qimage = QImage(thumb.tobytes("raw", "RGB"), thumb.width, thumb.height, thumb.width * 3, QImage.Format_RGB888)
        thumb_qimage = thumb_qimage.copy()
        thumbnail = QPixmap.fromImage(thumb_qimage)

        return PhotoEntry(path=str(path_obj), pil_image=pil_image, preview_qimage=preview_qimage, thumbnail=thumbnail)


class ThumbnailListWidget(QListWidget):
    photoActivated = Signal(str)
    photoDropped = Signal(list)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setViewMode(QListWidget.IconMode)
        self.setIconSize(THUMBNAIL_SIZE)
        self.setResizeMode(QListWidget.Adjust)
        self.setMovement(QListWidget.Static)
        self.setSpacing(8)
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        self.setAcceptDrops(True)
        self.viewport().setAcceptDrops(True)
        self.setDragEnabled(True)
        self._path_roles: Dict[str, QListWidgetItem] = {}
        self.itemClicked.connect(self._on_item_clicked)

    def add_photo(self, entry: PhotoEntry) -> None:
        if entry.path in self._path_roles:
            return
        item = QListWidgetItem(entry.thumbnail, Path(entry.path).name)
        item.setData(Qt.UserRole, entry.path)
        self.addItem(item)
        self._path_roles[entry.path] = item

    def remove_photos(self, paths: Iterable[str]) -> None:
        for path in list(paths):
            item = self._path_roles.pop(path, None)
            if item is None:
                continue
            row = self.row(item)
            self.takeItem(row)

    def _on_item_clicked(self, item: QListWidgetItem) -> None:
        path = item.data(Qt.UserRole)
        if path:
            self.photoActivated.emit(path)

    def mimeTypes(self) -> List[str]:
        return ["application/x-collage-path"]

    def mimeData(self, items: List[QListWidgetItem]) -> QMimeData:  # type: ignore[override]
        mime = QMimeData()
        paths = [item.data(Qt.UserRole) for item in items if item.data(Qt.UserRole)]
        mime.setData("application/x-collage-path", QByteArray("\n".join(paths).encode("utf-8")))
        return mime

    def startDrag(self, supportedActions: Qt.DropActions) -> None:  # type: ignore[override]
        items = self.selectedItems()
        if not items:
            return
        mime = self.mimeData(items)
        drag = QDrag(self)
        drag.setMimeData(mime)
        drag.exec(Qt.CopyAction)

    def dragEnterEvent(self, event: QDragEnterEvent) -> None:  # type: ignore[override]
        if event.mimeData().hasUrls() or event.mimeData().hasFormat("application/x-collage-path"):
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dragMoveEvent(self, event: QDragMoveEvent) -> None:  # type: ignore[override]
        if event.mimeData().hasUrls() or event.mimeData().hasFormat("application/x-collage-path"):
            event.acceptProposedAction()
        else:
            super().dragMoveEvent(event)

    def dropEvent(self, event: QDropEvent) -> None:  # type: ignore[override]
        paths: List[str] = []
        mime = event.mimeData()
        if mime.hasUrls():
            for url in mime.urls():
                if url.isLocalFile():
                    paths.append(url.toLocalFile())
        elif mime.hasFormat("application/x-collage-path"):
            data = bytes(mime.data("application/x-collage-path")).decode("utf-8")
            paths.extend(filter(None, data.split("\n")))
        if paths:
            self.photoDropped.emit(paths)
            event.acceptProposedAction()
        else:
            super().dropEvent(event)

    def clear_all(self) -> None:
        self._path_roles.clear()
        super().clear()


@dataclass
class CollageCellState:
    entry: Optional[PhotoEntry] = None
    scale: float = 1.0
    offset: QPointF = field(default_factory=QPointF)
    mode: str = "fill"  # or "fit"
    fit_scale: float = 1.0
    fill_scale: float = 1.0
    cell_size: QSizeF = field(default_factory=lambda: QSizeF(1.0, 1.0))

    def set_entry(self, entry: Optional[PhotoEntry]) -> None:
        self.entry = entry
        self.scale = 1.0
        self.offset = QPointF()
        self.mode = "fill"
        if entry:
            self._recompute_scales()
            self.scale = self.fill_scale
        else:
            self.fit_scale = 1.0
            self.fill_scale = 1.0

    def set_cell_size(self, size: QSizeF) -> None:
        old_size = self.cell_size
        if old_size.width() > 0 and old_size.height() > 0:
            if size.width() > 0:
                self.offset.setX(self.offset.x() * size.width() / old_size.width())
            if size.height() > 0:
                self.offset.setY(self.offset.y() * size.height() / old_size.height())
        self.cell_size = size
        if self.entry:
            self._recompute_scales(adjust_current=True)

    def _recompute_scales(self, adjust_current: bool = False) -> None:
        if not self.entry:
            return
        image = self.entry.preview_qimage
        if image.isNull():
            return
        cell_w = max(1.0, self.cell_size.width())
        cell_h = max(1.0, self.cell_size.height())
        img_w = max(1, image.width())
        img_h = max(1, image.height())
        fit_scale = min(cell_w / img_w, cell_h / img_h)
        fill_scale = max(cell_w / img_w, cell_h / img_h)
        self.fit_scale = fit_scale
        self.fill_scale = fill_scale
        if adjust_current:
            if self.mode == "fill":
                self.scale = max(self.scale, self.fill_scale)
            else:
                if self.scale < self.fit_scale:
                    self.scale = self.fit_scale
                elif self.scale > self.fill_scale * 5:
                    self.scale = self.fill_scale * 5
        self.clamp_offset()

    def clamp_offset(self) -> None:
        if not self.entry:
            self.offset = QPointF()
            return
        image = self.entry.preview_qimage
        cell_w = self.cell_size.width()
        cell_h = self.cell_size.height()
        scaled_w = image.width() * self.scale
        scaled_h = image.height() * self.scale
        half_cell_w = cell_w / 2.0
        half_cell_h = cell_h / 2.0
        half_scaled_w = scaled_w / 2.0
        half_scaled_h = scaled_h / 2.0
        if self.mode == "fill":
            limit_x = max(0.0, half_scaled_w - half_cell_w)
            limit_y = max(0.0, half_scaled_h - half_cell_h)
        else:
            limit_x = max(0.0, half_cell_w - half_scaled_w)
            limit_y = max(0.0, half_cell_h - half_scaled_h)
        limit_x = max(limit_x, 0.0)
        limit_y = max(limit_y, 0.0)
        self.offset.setX(max(-limit_x, min(limit_x, self.offset.x())))
        self.offset.setY(max(-limit_y, min(limit_y, self.offset.y())))

    def pan(self, delta: QPointF) -> None:
        self.offset = QPointF(self.offset.x() + delta.x(), self.offset.y() + delta.y())
        self.clamp_offset()

    def zoom(self, factor: float, anchor: QPointF) -> None:
        if not self.entry:
            return
        old_scale = self.scale
        new_scale = self.scale * factor
        min_scale = max(self.fit_scale * 0.2, 0.02)
        max_scale = self.fill_scale * 6
        new_scale = max(min_scale, min(max_scale, new_scale))
        if abs(new_scale - old_scale) < 1e-5:
            return
        cell_center_x = self.cell_size.width() / 2.0
        cell_center_y = self.cell_size.height() / 2.0
        anchor_rel_x = anchor.x() - cell_center_x
        anchor_rel_y = anchor.y() - cell_center_y
        scale_ratio = new_scale / old_scale
        offset_x = anchor_rel_x + (self.offset.x() - anchor_rel_x) * scale_ratio
        offset_y = anchor_rel_y + (self.offset.y() - anchor_rel_y) * scale_ratio
        self.offset = QPointF(offset_x, offset_y)
        self.scale = new_scale
        self.clamp_offset()

    def set_mode(self, mode: str) -> None:
        if mode not in {"fill", "fit"}:
            return
        self.mode = mode
        if mode == "fill":
            self.scale = max(self.scale, self.fill_scale)
        else:
            self.scale = max(self.fit_scale, min(self.scale, self.fill_scale * 6))
        self.offset = QPointF()
        self.clamp_offset()

    def reset_view(self) -> None:
        if not self.entry:
            return
        if self.mode == "fill":
            self.scale = self.fill_scale
        else:
            self.scale = self.fit_scale
        self.offset = QPointF()
        self.clamp_offset()


class CollageCellItem(QGraphicsObject):
    Type = QGraphicsItem.UserType + 1

    def __init__(self, index: int, rect: QRectF, state: CollageCellState):
        super().__init__()
        self.index = index
        self._size = QSizeF(rect.width(), rect.height())
        self.state = state
        self.setAcceptHoverEvents(True)
        self.setAcceptedMouseButtons(Qt.LeftButton | Qt.RightButton)
        self.setFlag(QGraphicsItem.ItemClipsChildrenToShape, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setZValue(1)
        self._hover = False
        self.setPos(rect.topLeft())
        self.state.set_cell_size(self._size)

    def type(self) -> int:  # type: ignore[override]
        return CollageCellItem.Type

    def boundingRect(self) -> QRectF:  # type: ignore[override]
        return QRectF(0.0, 0.0, self._size.width(), self._size.height())

    def set_rect(self, rect: QRectF) -> None:
        self.prepareGeometryChange()
        self._size = QSizeF(rect.width(), rect.height())
        self.setPos(rect.topLeft())
        self.state.set_cell_size(self._size)
        self.update()

    def paint(self, painter: QPainter, option: QStyleOptionGraphicsItem, widget: Optional[QWidget] = None) -> None:  # type: ignore[override]
        rect = self.boundingRect()
        painter.save()
        painter.fillRect(rect, QColor(255, 255, 255))
        border_color = QColor(200, 200, 200) if self._hover else QColor(150, 150, 150)
        pen = QPen(border_color, 2)
        painter.setPen(pen)
        painter.drawRect(rect)
        if self.state.entry and not self.state.entry.preview_qimage.isNull():
            painter.setClipRect(rect.adjusted(2, 2, -2, -2))
            image = self.state.entry.preview_qimage
            scaled_w = image.width() * self.state.scale
            scaled_h = image.height() * self.state.scale
            center = QPointF(rect.width() / 2.0 + self.state.offset.x(),
                             rect.height() / 2.0 + self.state.offset.y())
            top_left = QPointF(center.x() - scaled_w / 2.0, center.y() - scaled_h / 2.0)
            target = QRectF(top_left, QSizeF(scaled_w, scaled_h))
            painter.drawImage(target, image)
        elif not self.state.entry:
            painter.setPen(QPen(QColor(180, 180, 180), 1, Qt.DashLine))
            painter.drawText(rect, Qt.AlignCenter, "Drop Photo")
        painter.restore()

    def hoverEnterEvent(self, event: QGraphicsSceneHoverEvent) -> None:  # type: ignore[override]
        self._hover = True
        self.update()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event: QGraphicsSceneHoverEvent) -> None:  # type: ignore[override]
        self._hover = False
        self.update()
        super().hoverLeaveEvent(event)


class CollageScene(QGraphicsScene):
    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)


class CollageView(QGraphicsView):
    cellActivated = Signal(int)
    cellDropPaths = Signal(int, list)
    cellSwapRequested = Signal(int, int)
    cellContextMenuRequested = Signal(int, QPointF)
    cellToggleModeRequested = Signal(int)
    cellPanRequested = Signal(int, QPointF)
    cellZoomRequested = Signal(int, float, QPointF)
    cellPanStarted = Signal(int)
    cellPanFinished = Signal(int)

    def __init__(self, scene: QGraphicsScene, parent: Optional[QWidget] = None) -> None:
        super().__init__(scene, parent)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setMouseTracking(True)
        self.setAcceptDrops(True)
        self._cells: List[CollageCellItem] = []
        self._pressed_cell: Optional[CollageCellItem] = None
        self._press_scene_pos = QPointF()
        self._last_scene_pos = QPointF()
        self._pan_active = False
        self._drag_started = False
        self._cursor_override = False

    def set_cells(self, cells: List[CollageCellItem]) -> None:
        self._cells = cells

    def fit_view(self) -> None:
        if not self._cells:
            return
        rect = self.sceneRect()
        if rect.isEmpty():
            return
        self.fitInView(rect, Qt.KeepAspectRatio)

    def _cell_from_scene_pos(self, pos: QPointF) -> Optional[CollageCellItem]:
        items = self.scene().items(pos)
        for item in items:
            if isinstance(item, CollageCellItem):
                return item
        return None

    def mousePressEvent(self, event: QMouseEvent) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.position().toPoint())
            cell = self._cell_from_scene_pos(scene_pos)
            if cell:
                self._pressed_cell = cell
                self._press_scene_pos = scene_pos
                self._last_scene_pos = scene_pos
                self._pan_active = False
                self._drag_started = False
                self.cellActivated.emit(cell.index)
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:  # type: ignore[override]
        if self._pressed_cell and not self._drag_started:
            scene_pos = self.mapToScene(event.position().toPoint())
            delta = scene_pos - self._press_scene_pos
            distance = (delta.x() ** 2 + delta.y() ** 2) ** 0.5
            if not self._pan_active and distance > 4:
                local_pos = self._pressed_cell.mapFromScene(scene_pos)
                local_press = self._pressed_cell.mapFromScene(self._press_scene_pos)
                if self._pressed_cell.contains(local_pos):
                    self._pan_active = True
                    self.cellPanStarted.emit(self._pressed_cell.index)
                else:
                    self._start_drag_from_cell(self._pressed_cell)
                    self._drag_started = True
                    return
            if self._pan_active:
                prev_local = self._pressed_cell.mapFromScene(self._last_scene_pos)
                current_local = self._pressed_cell.mapFromScene(scene_pos)
                delta_local = current_local - prev_local
                self.cellPanRequested.emit(self._pressed_cell.index, delta_local)
                self._last_scene_pos = scene_pos
                return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            if self._pan_active and self._pressed_cell:
                self.cellPanFinished.emit(self._pressed_cell.index)
            self._pressed_cell = None
            self._pan_active = False
            self._drag_started = False
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event: QWheelEvent) -> None:  # type: ignore[override]
        if QApplication.keyboardModifiers() & Qt.ControlModifier:
            super().wheelEvent(event)
            return
        scene_pos = self.mapToScene(event.position().toPoint())
        cell = self._cell_from_scene_pos(scene_pos)
        if not cell:
            super().wheelEvent(event)
            return
        angle_delta = event.angleDelta().y()
        if angle_delta == 0:
            return
        factor = 1.0 + (0.1 if angle_delta > 0 else -0.1)
        anchor_local = cell.mapFromScene(scene_pos)
        self.cellZoomRequested.emit(cell.index, factor, anchor_local)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:  # type: ignore[override]
        scene_pos = self.mapToScene(event.position().toPoint())
        cell = self._cell_from_scene_pos(scene_pos)
        if cell:
            self.cellToggleModeRequested.emit(cell.index)
            return
        super().mouseDoubleClickEvent(event)

    def resizeEvent(self, event: QResizeEvent) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        self.fit_view()

    def contextMenuEvent(self, event: QContextMenuEvent) -> None:  # type: ignore[override]
        scene_pos = self.mapToScene(event.pos())
        cell = self._cell_from_scene_pos(scene_pos)
        if cell:
            self.cellContextMenuRequested.emit(cell.index, event.globalPos())
            return
        super().contextMenuEvent(event)

    def dragEnterEvent(self, event: QDragEnterEvent) -> None:  # type: ignore[override]
        if event.mimeData().hasUrls() or event.mimeData().hasFormat("application/x-collage-path") or event.mimeData().hasFormat("application/x-collage-cell"):
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dragMoveEvent(self, event: QDragMoveEvent) -> None:  # type: ignore[override]
        if event.mimeData().hasUrls() or event.mimeData().hasFormat("application/x-collage-path") or event.mimeData().hasFormat("application/x-collage-cell"):
            event.acceptProposedAction()
        else:
            super().dragMoveEvent(event)

    def dropEvent(self, event: QDropEvent) -> None:  # type: ignore[override]
        scene_pos = self.mapToScene(event.position().toPoint())
        cell = self._cell_from_scene_pos(scene_pos)
        if not cell:
            super().dropEvent(event)
            return
        mime = event.mimeData()
        if mime.hasFormat("application/x-collage-path"):
            data = bytes(mime.data("application/x-collage-path")).decode("utf-8")
            paths = list(filter(None, data.split("\n")))
            if paths:
                self.cellDropPaths.emit(cell.index, paths)
                event.acceptProposedAction()
                return
        elif mime.hasUrls():
            paths = [url.toLocalFile() for url in mime.urls() if url.isLocalFile()]
            if paths:
                self.cellDropPaths.emit(cell.index, paths)
                event.acceptProposedAction()
                return
        elif mime.hasFormat("application/x-collage-cell"):
            data = bytes(mime.data("application/x-collage-cell")).decode("utf-8")
            try:
                src_index = int(data)
                if src_index != cell.index:
                    self.cellSwapRequested.emit(src_index, cell.index)
                    event.acceptProposedAction()
                    return
            except ValueError:
                pass
        super().dropEvent(event)

    def _start_drag_from_cell(self, cell: CollageCellItem) -> None:
        if not cell.state.entry:
            return
        mime = QMimeData()
        mime.setData("application/x-collage-cell", QByteArray(str(cell.index).encode("utf-8")))
        drag = QDrag(self)
        drag.setMimeData(mime)
        drag.exec(Qt.MoveAction)


class SettingsDialog(QDialog):
    settingsChanged = Signal(CollageSettings)

    def __init__(self, settings: CollageSettings, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Export Settings")
        self._settings = dataclasses.replace(settings)
        layout = QVBoxLayout(self)

        form = QFormLayout()
        self.long_side_spin = QSpinBox()
        self.long_side_spin.setRange(3000, 8000)
        self.long_side_spin.setSingleStep(500)
        self.long_side_spin.setValue(self._settings.long_side)
        form.addRow("Long edge (px)", self.long_side_spin)

        self.dpi_spin = QSpinBox()
        self.dpi_spin.setRange(72, 600)
        self.dpi_spin.setValue(self._settings.dpi)
        form.addRow("DPI", self.dpi_spin)

        self.format_combo = QComboBoxWithValues(["JPEG", "PNG"], self._settings.image_format)
        form.addRow("Default format", self.format_combo)

        self.quality_slider = QSlider(Qt.Horizontal)
        self.quality_slider.setRange(90, 100)
        self.quality_slider.setValue(self._settings.jpeg_quality)
        form.addRow("JPEG quality", self.quality_slider)

        self.border_spin = QSpinBox()
        self.border_spin.setRange(0, 200)
        self.border_spin.setValue(self._settings.border_width)
        form.addRow("Border width (px)", self.border_spin)

        self.remove_checkbox = QCheckBoxWithState("Remove used photos on save", self._settings.remove_used)
        form.addRow("", self.remove_checkbox)

        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, Qt.Horizontal, self)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def accept(self) -> None:  # type: ignore[override]
        self._settings.long_side = self.long_side_spin.value()
        self._settings.dpi = self.dpi_spin.value()
        self._settings.image_format = self.format_combo.currentText()
        self._settings.jpeg_quality = self.quality_slider.value()
        self._settings.border_width = self.border_spin.value()
        self._settings.remove_used = self.remove_checkbox.isChecked()
        self.settingsChanged.emit(self._settings)
        super().accept()


class QComboBoxWithValues(QComboBox):
    def __init__(self, values: Sequence[str], current: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.addItems(values)
        idx = self.findText(current, Qt.MatchFixedString)
        if idx >= 0:
            self.setCurrentIndex(idx)


class QCheckBoxWithState(QCheckBox):
    def __init__(self, text: str, checked: bool, parent: Optional[QWidget] = None) -> None:
        super().__init__(text, parent)
        self.setChecked(checked)


class CollageStateSnapshot:
    def __init__(self, states: List[CollageCellState]) -> None:
        self.cells = [self._copy_state(state) for state in states]

    @staticmethod
    def _copy_state(state: CollageCellState) -> CollageCellState:
        new_state = CollageCellState()
        new_state.entry = state.entry
        new_state.scale = state.scale
        new_state.offset = QPointF(state.offset)
        new_state.mode = state.mode
        new_state.fit_scale = state.fit_scale
        new_state.fill_scale = state.fill_scale
        new_state.cell_size = QSizeF(state.cell_size)
        return new_state

    def restore(self, target_states: List[CollageCellState]) -> None:
        for idx, snapshot in enumerate(self.cells):
            if idx >= len(target_states):
                break
            target = target_states[idx]
            target.entry = snapshot.entry
            target.scale = snapshot.scale
            target.offset = QPointF(snapshot.offset)
            target.mode = snapshot.mode
            target.fit_scale = snapshot.fit_scale
            target.fill_scale = snapshot.fill_scale
            target.cell_size = QSizeF(snapshot.cell_size)
            target.clamp_offset()


class ToastLabel(QLabel):
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.setStyleSheet("""
            background-color: rgba(30, 30, 30, 220);
            color: white;
            padding: 6px 12px;
            border-radius: 6px;
        """)
        self.setWindowFlags(Qt.ToolTip)
        self.setAlignment(Qt.AlignCenter)
        self.hide()
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self.hide)

    def show_message(self, text: str, duration_ms: int = 2000) -> None:
        self.setText(text)
        self.adjustSize()
        parent_rect = self.parentWidget().rect()
        geo = self.geometry()
        geo.moveCenter(parent_rect.center())
        self.setGeometry(geo)
        self.show()
        self.raise_()
        self._timer.start(duration_ms)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Collage Maker")
        self.resize(1300, 800)
        self.config = ConfigManager()
        self.settings = self.config.settings
        self.thread_pool = QThreadPool.globalInstance()

        self.photo_entries: Dict[str, PhotoEntry] = {}
        self.cell_states = [CollageCellState() for _ in range(4)]
        self._undo_stack: List[CollageStateSnapshot] = []
        self._active_cell_index: Optional[int] = None
        self._active_thumbnail_path: Optional[str] = None
        self.orientation = "landscape"
        self._pending_assignments: Dict[str, List[int]] = {}
        self._pan_snapshot_index: Optional[int] = None
        self._loading_paths: set[str] = set()

        self._init_ui()
        self._rebuild_scene()
        self._toast = ToastLabel(self)

    def _init_ui(self) -> None:
        self._init_actions()
        toolbar = QToolBar("Main Toolbar")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        toolbar.addAction(self.open_action)
        toolbar.addAction(self.save_action)
        toolbar.addAction(self.undo_action)
        toolbar.addAction(self.clear_action)
        toolbar.addSeparator()
        toolbar.addAction(self.fit_view_action)
        toolbar.addAction(self.orientation_action)
        toolbar.addSeparator()
        toolbar.addAction(self.settings_action)

        splitter = QSplitter(self)
        self.setCentralWidget(splitter)

        self.thumbnail_list = ThumbnailListWidget()
        self.thumbnail_list.photoActivated.connect(self._on_thumbnail_activated)
        self.thumbnail_list.photoDropped.connect(self._on_paths_dropped)

        thumbnail_panel = QWidget()
        thumb_layout = QVBoxLayout(thumbnail_panel)
        thumb_layout.setContentsMargins(6, 6, 6, 6)
        thumb_layout.addWidget(QLabel("Thumbnail Buffer"))
        controls_layout = QHBoxLayout()
        self.sort_button = QPushButton("Sort A→Z")
        self.sort_button.clicked.connect(self._sort_gallery_by_name)
        controls_layout.addWidget(self.sort_button)
        self.clear_buffer_button = QPushButton("Clear Buffer")
        self.clear_buffer_button.clicked.connect(self._clear_gallery_buffer)
        controls_layout.addWidget(self.clear_buffer_button)
        controls_layout.addStretch(1)
        thumb_layout.addLayout(controls_layout)
        thumb_layout.addWidget(self.thumbnail_list)
        self.remaining_label = QLabel("0 photos")
        thumb_layout.addWidget(self.remaining_label)

        splitter.addWidget(thumbnail_panel)

        self.scene = CollageScene(self)
        self.view = CollageView(self.scene, self)
        self.view.cellActivated.connect(self._on_cell_activated)
        self.view.cellDropPaths.connect(self._on_paths_dropped_into_cell)
        self.view.cellSwapRequested.connect(self._swap_cells)
        self.view.cellContextMenuRequested.connect(self._show_cell_menu)
        self.view.cellToggleModeRequested.connect(self._toggle_cell_mode)
        self.view.cellPanRequested.connect(self._pan_cell)
        self.view.cellZoomRequested.connect(self._zoom_cell)
        self.view.cellPanStarted.connect(self._on_pan_started)
        self.view.cellPanFinished.connect(self._on_pan_finished)
        self.fit_view_action.triggered.connect(self.view.fit_view)
        splitter.addWidget(self.view)
        splitter.setStretchFactor(1, 1)

        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Load photos to begin")

        self._update_remaining_label()

    def _init_actions(self) -> None:
        style = self.style()

        self.open_action = QAction(QIcon.fromTheme("document-open", style.standardIcon(QStyle.SP_DialogOpenButton)), "Load Photos…", self)
        self.open_action.setShortcut(QKeySequence.Open)
        self.open_action.triggered.connect(self._load_photos_dialog)

        self.save_action = QAction(QIcon.fromTheme("document-save", style.standardIcon(QStyle.SP_DialogSaveButton)), "Save Collage", self)
        self.save_action.setShortcut(QKeySequence.Save)
        self.save_action.triggered.connect(self._save_collage)

        self.undo_action = QAction(QIcon.fromTheme("edit-undo", style.standardIcon(QStyle.SP_ArrowBack)), "Undo", self)
        self.undo_action.setShortcut(QKeySequence.Undo)
        self.undo_action.triggered.connect(self._undo)
        self.undo_action.setEnabled(False)

        self.clear_action = QAction(QIcon.fromTheme("edit-clear", style.standardIcon(QStyle.SP_DialogResetButton)), "Clear Collage", self)
        self.clear_action.triggered.connect(self._clear_collage)

        self.fit_view_action = QAction("Fit View", self)

        self.orientation_action = QAction("Orientation: Landscape", self)
        self.orientation_action.setCheckable(True)
        self.orientation_action.triggered.connect(self._toggle_orientation)

        self.settings_action = QAction(QIcon.fromTheme("preferences-system"), "Settings", self)
        self.settings_action.triggered.connect(self._open_settings)

    def _rebuild_scene(self) -> None:
        self.scene.clear()
        canvas_rect = self._canvas_rect()
        self.scene.setSceneRect(canvas_rect)
        cells: List[CollageCellItem] = []
        columns = 2
        rows = 2
        cell_width = canvas_rect.width() / columns
        cell_height = canvas_rect.height() / rows
        for i in range(rows):
            for j in range(columns):
                index = i * columns + j
                rect = QRectF(canvas_rect.x() + j * cell_width,
                              canvas_rect.y() + i * cell_height,
                              cell_width, cell_height)
                state = self.cell_states[index]
                cell_item = CollageCellItem(index, rect, state)
                self.scene.addItem(cell_item)
                cells.append(cell_item)
        self.view.set_cells(cells)
        self.view.fit_view()

    def _canvas_rect(self) -> QRectF:
        if self.orientation == "landscape":
            width = 600.0
            height = 400.0
        else:
            width = 400.0
            height = 600.0
        return QRectF(0.0, 0.0, width, height)

    def _toggle_orientation(self) -> None:
        self.orientation = "portrait" if self.orientation == "landscape" else "landscape"
        self.orientation_action.setText(f"Orientation: {'Landscape' if self.orientation == 'landscape' else 'Portrait'}")
        self._snapshot_for_undo()
        self._rebuild_scene()
        self.status_bar.showMessage(f"Orientation set to {self.orientation}", 2000)

    def _open_settings(self) -> None:
        dialog = SettingsDialog(self.settings, self)
        dialog.settingsChanged.connect(self._apply_settings)
        dialog.exec()

    def _apply_settings(self, settings: CollageSettings) -> None:
        self.settings = dataclasses.replace(settings)
        self.config.settings = dataclasses.replace(settings)
        self.config.save()
        self.status_bar.showMessage("Settings updated", 2000)

    def _sort_gallery_by_name(self) -> None:
        self.thumbnail_list.sortItems(Qt.AscendingOrder)
        self.status_bar.showMessage("Gallery sorted by filename", 2000)

    def _clear_gallery_buffer(self) -> None:
        if not self.photo_entries:
            return
        reply = QMessageBox.question(self, "Clear buffer", "Remove all photos from the buffer?", QMessageBox.Yes | QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        self.thumbnail_list.clear_all()
        self.photo_entries.clear()
        self._loading_paths.clear()
        self._pending_assignments.clear()
        self._active_thumbnail_path = None
        self._update_remaining_label()
        self.status_bar.showMessage("Thumbnail buffer cleared", 3000)

    def _load_photos_dialog(self) -> None:
        start_dir = self.config.last_open_dir or str(Path.home())
        paths, _ = QFileDialog.getOpenFileNames(self, "Select photos", start_dir, "Images (*.jpg *.jpeg *.png *.bmp *.tif *.tiff *.heic *.heif)")
        if not paths:
            return
        self.config.last_open_dir = str(Path(paths[0]).parent)
        self.config.save()
        self._enqueue_paths(paths)

    def _enqueue_paths(self, paths: Iterable[str]) -> None:
        unique_paths: List[str] = []
        for path in paths:
            normalized = str(Path(path))
            ext = Path(normalized).suffix.lower()
            if ext not in SUPPORTED_EXTENSIONS:
                continue
            if normalized in self.photo_entries or normalized in self._loading_paths:
                continue
            unique_paths.append(normalized)
        if not unique_paths:
            return
        signals = WorkerSignals()
        signals.finished.connect(self._on_photo_loaded)
        signals.failed.connect(self._on_photo_failed)
        for path in unique_paths:
            self._loading_paths.add(path)
            worker = ThumbnailWorker(path, signals)
            self.thread_pool.start(worker)

    def _on_photo_loaded(self, path: str, entry: PhotoEntry) -> None:
        self._loading_paths.discard(path)
        self.photo_entries[path] = entry
        self.thumbnail_list.add_photo(entry)
        self._update_remaining_label()
        pending = self._pending_assignments.pop(path, [])
        for index in pending:
            self._assign_photo_to_cell(index, path)

    def _on_photo_failed(self, path: str, error: str) -> None:
        self._loading_paths.discard(path)
        self.status_bar.showMessage(f"Failed to load {path}: {error}", 5000)
        if path in self._pending_assignments:
            self._pending_assignments.pop(path, None)

    def _on_paths_dropped(self, paths: Sequence[str]) -> None:
        self._enqueue_paths(paths)

    def _on_thumbnail_activated(self, path: str) -> None:
        self._active_thumbnail_path = path
        self.status_bar.showMessage(f"Selected {Path(path).name}. Click a cell to place.", 3000)

    def _on_cell_activated(self, index: int) -> None:
        self._active_cell_index = index
        if self._active_thumbnail_path:
            self._assign_photo_to_cell(index, self._active_thumbnail_path)

    def _on_paths_dropped_into_cell(self, index: int, paths: Sequence[str]) -> None:
        normalized = [str(Path(path)) for path in paths if path]
        if not normalized:
            return
        for path in normalized:
            if path in self.photo_entries:
                self._assign_photo_to_cell(index, path)
                return
        target_path = normalized[0]
        self._pending_assignments.setdefault(target_path, []).append(index)
        self._enqueue_paths(normalized)

    def _assign_photo_to_cell(self, index: int, path: str) -> None:
        entry = self.photo_entries.get(path)
        if not entry:
            return
        self._snapshot_for_undo()
        self.cell_states[index].set_entry(entry)
        self.view.viewport().update()
        self.status_bar.showMessage(f"Placed {Path(path).name} in cell {index + 1}", 2000)
        self._active_thumbnail_path = None
        self.thumbnail_list.clearSelection()

    def _toggle_cell_mode(self, index: int) -> None:
        cell_state = self.cell_states[index]
        if not cell_state.entry:
            return
        self._snapshot_for_undo()
        new_mode = "fit" if cell_state.mode == "fill" else "fill"
        cell_state.set_mode(new_mode)
        self.view.viewport().update()

    def _show_cell_menu(self, index: int, global_pos: QPointF) -> None:
        menu = QMenu(self)
        fill_action = menu.addAction("Fill")
        fit_action = menu.addAction("Fit")
        reset_action = menu.addAction("Reset View")
        replace_action = menu.addAction("Replace…")
        clear_action = menu.addAction("Clear Cell")
        action = menu.exec(QPoint(int(global_pos.x()), int(global_pos.y())))
        if action == fill_action:
            self._snapshot_for_undo()
            self.cell_states[index].set_mode("fill")
        elif action == fit_action:
            self._snapshot_for_undo()
            self.cell_states[index].set_mode("fit")
        elif action == reset_action:
            self._snapshot_for_undo()
            self.cell_states[index].reset_view()
        elif action == replace_action:
            self._replace_cell_image(index)
        elif action == clear_action:
            self._snapshot_for_undo()
            self.cell_states[index].set_entry(None)
        self.view.viewport().update()

    def _replace_cell_image(self, index: int) -> None:
        start_dir = self.config.last_open_dir or str(Path.home())
        path, _ = QFileDialog.getOpenFileName(self, "Replace photo", start_dir, "Images (*.jpg *.jpeg *.png *.bmp *.tif *.tiff *.heic *.heif)")
        if not path:
            return
        self.config.last_open_dir = str(Path(path).parent)
        self.config.save()
        normalized = str(Path(path))
        if normalized in self.photo_entries:
            self._assign_photo_to_cell(index, normalized)
            return
        self._pending_assignments.setdefault(normalized, []).append(index)
        self._enqueue_paths([normalized])

    def _pan_cell(self, index: int, delta: QPointF) -> None:
        state = self.cell_states[index]
        if not state.entry:
            return
        state.pan(delta)
        self.view.viewport().update()

    def _zoom_cell(self, index: int, factor: float, anchor: QPointF) -> None:
        state = self.cell_states[index]
        if not state.entry:
            return
        self._snapshot_for_undo()
        state.zoom(factor, anchor)
        self.view.viewport().update()

    def _on_pan_started(self, index: int) -> None:
        state = self.cell_states[index]
        if not state.entry:
            return
        if self._pan_snapshot_index is None:
            self._snapshot_for_undo()
            self._pan_snapshot_index = index

    def _on_pan_finished(self, index: int) -> None:
        if self._pan_snapshot_index == index:
            self._pan_snapshot_index = None

    def _swap_cells(self, src: int, dst: int) -> None:
        self._snapshot_for_undo()
        self.cell_states[src], self.cell_states[dst] = self.cell_states[dst], self.cell_states[src]
        self._rebuild_scene()
        self.view.viewport().update()

    def _clear_collage(self) -> None:
        self._snapshot_for_undo()
        for state in self.cell_states:
            state.set_entry(None)
        self.view.viewport().update()

    def _snapshot_for_undo(self) -> None:
        snapshot = CollageStateSnapshot(self.cell_states)
        self._undo_stack.append(snapshot)
        if len(self._undo_stack) > UNDO_HISTORY_LIMIT:
            self._undo_stack.pop(0)
        self.undo_action.setEnabled(True)

    def _undo(self) -> None:
        if not self._undo_stack:
            return
        snapshot = self._undo_stack.pop()
        snapshot.restore(self.cell_states)
        self._rebuild_scene()
        self.view.viewport().update()
        if not self._undo_stack:
            self.undo_action.setEnabled(False)

    def _update_remaining_label(self) -> None:
        count = len(self.photo_entries)
        self.remaining_label.setText(f"{count} photos")

    def _save_collage(self) -> None:
        filled_paths = [state.entry.path for state in self.cell_states if state.entry]
        if len(filled_paths) < 4:
            reply = QMessageBox.question(self, "Incomplete collage", "Not all cells are filled. Continue and leave blanks?", QMessageBox.Yes | QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        long_side = self.settings.long_side
        width, height = self._export_dimensions(long_side)
        image = Image.new("RGB", (width, height), color=(255, 255, 255))
        cell_w = width // 2
        cell_h = height // 2
        border_width = max(0, int(self.settings.border_width))
        arranger = [
            (0, (0, 0)),
            (1, (cell_w, 0)),
            (2, (0, cell_h)),
            (3, (cell_w, cell_h)),
        ]
        for index, (x, y) in arranger:
            state = self.cell_states[index]
            target_box = (x, y, x + cell_w, y + cell_h)
            self._render_cell_to_image(image, target_box, state)

        if border_width > 0:
            self._draw_collage_borders(image, border_width)

        default_format = self.settings.image_format
        ext = "jpg" if default_format.upper() == "JPEG" else "png"
        suggested = Path(self.config.last_save_dir or Path.home()).joinpath(f"collage_{time.strftime('%Y%m%d_%H%M%S')}.{ext}")
        file_path, selected_filter = QFileDialog.getSaveFileName(self, "Save collage", str(suggested), "JPEG (*.jpg *.jpeg);;PNG (*.png)")
        if not file_path:
            return
        dest_path = Path(file_path)
        self.config.last_save_dir = str(dest_path.parent)
        self.config.save()

        selected_format = "PNG" if dest_path.suffix.lower() == ".png" or "PNG" in selected_filter else "JPEG"
        save_kwargs = {"dpi": (self.settings.dpi, self.settings.dpi)}
        if selected_format == "JPEG":
            save_kwargs["quality"] = self.settings.jpeg_quality
            save_kwargs["subsampling"] = 0
        image.save(dest_path, selected_format, **save_kwargs)
        self._toast.show_message("Collage saved")
        if self.settings.remove_used and filled_paths:
            self.thumbnail_list.remove_photos(filled_paths)
            for path in filled_paths:
                self.photo_entries.pop(path, None)
            self._update_remaining_label()

    def _export_dimensions(self, long_side: int) -> Tuple[int, int]:
        if self.orientation == "landscape":
            width = long_side
            height = int(round(long_side * 2 / 3))
        else:
            height = long_side
            width = int(round(long_side * 2 / 3))
        return width, height

    def _render_cell_to_image(self, canvas: Image.Image, target_box: Tuple[int, int, int, int], state: CollageCellState) -> None:
        x0, y0, x1, y1 = target_box
        cell_width = x1 - x0
        cell_height = y1 - y0
        if not state.entry:
            return
        pil_image = state.entry.pil_image
        img_w, img_h = pil_image.size
        preview = state.entry.preview_qimage
        if preview.isNull():
            return
        view_cell_w = max(1.0, state.cell_size.width())
        view_cell_h = max(1.0, state.cell_size.height())
        preview_w = preview.width()
        preview_h = preview.height()

        display_width_view = preview_w * state.scale
        display_height_view = preview_h * state.scale
        ratio_w = display_width_view / view_cell_w
        ratio_h = display_height_view / view_cell_h
        export_cell_w = float(cell_width)
        export_cell_h = float(cell_height)
        display_width_export = ratio_w * export_cell_w
        display_height_export = ratio_h * export_cell_h
        if display_width_export <= 0 or display_height_export <= 0:
            return

        offset_export_x = state.offset.x() * (export_cell_w / view_cell_w)
        offset_export_y = state.offset.y() * (export_cell_h / view_cell_h)
        center_x = export_cell_w / 2.0 + offset_export_x
        center_y = export_cell_h / 2.0 + offset_export_y
        dest_rect = (
            center_x - display_width_export / 2.0,
            center_y - display_height_export / 2.0,
            center_x + display_width_export / 2.0,
            center_y + display_height_export / 2.0,
        )

        inter_left = max(0.0, dest_rect[0])
        inter_top = max(0.0, dest_rect[1])
        inter_right = min(export_cell_w, dest_rect[2])
        inter_bottom = min(export_cell_h, dest_rect[3])
        if inter_left >= inter_right or inter_top >= inter_bottom:
            return
        # Map intersection back to source coordinates
        dest_width = dest_rect[2] - dest_rect[0]
        dest_height = dest_rect[3] - dest_rect[1]
        src_left = (inter_left - dest_rect[0]) * img_w / dest_width
        src_top = (inter_top - dest_rect[1]) * img_h / dest_height
        src_right = (inter_right - dest_rect[0]) * img_w / dest_width
        src_bottom = (inter_bottom - dest_rect[1]) * img_h / dest_height
        src_box = (int(src_left), int(src_top), int(src_right), int(src_bottom))
        cropped = pil_image.crop(src_box)
        dest_width = int(round(inter_right - inter_left))
        dest_height = int(round(inter_bottom - inter_top))
        if dest_width <= 0 or dest_height <= 0:
            return
        resized = cropped.resize((dest_width, dest_height), Image.Resampling.LANCZOS)
        paste_x = x0 + int(round(inter_left))
        paste_y = y0 + int(round(inter_top))
        canvas.paste(resized, (paste_x, paste_y))

    def _draw_collage_borders(self, canvas: Image.Image, border_width: int) -> None:
        draw = ImageDraw.Draw(canvas)
        width, height = canvas.size
        color = (190, 190, 190)
        # Outer rectangle
        inset = border_width / 2
        draw.rectangle([(inset, inset),
                        (width - inset - 1, height - inset - 1)],
                       outline=color, width=border_width)
        # Vertical divider
        mid_x = width / 2
        draw.line([(mid_x, 0), (mid_x, height)], fill=color, width=border_width)
        # Horizontal divider
        mid_y = height / 2
        draw.line([(0, mid_y), (width, mid_y)], fill=color, width=border_width)

def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
