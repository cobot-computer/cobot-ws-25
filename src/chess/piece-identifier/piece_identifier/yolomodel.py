from ament_index_python.packages import get_package_share_directory
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
from ultralytics import YOLO
import os

TEXT_SIZE = 8
DEBUG_MUL = 1

B_PERCENT = 22 / 377
RESIZE_TO = (50, 50)


def draw_bb(image, left, upper, right, lower, label):
    # Create a drawing context
    draw = ImageDraw.Draw(image)

    # Draw the rectangle (bounding box)
    draw.rectangle((left, upper, right, lower), outline="red", width=3)

    try:
        font = ImageFont.truetype("arial.ttf", TEXT_SIZE * DEBUG_MUL)
    except IOError:
        font = ImageFont.load_default()

    # Position for the label. This example positions it above the bounding box.
    label_pos = (left, upper - TEXT_SIZE * DEBUG_MUL - 2)

    # Draw the label
    draw.text(label_pos, label, fill="cyan", font=font)

    return image


class YOLOModel:
    def __init__(self, model_path):
        # Load the model
        package_share_directory = get_package_share_directory("piece_identifier")
        self.model = YOLO(os.path.join(package_share_directory, model_path))

    def predict(self, img):
        # Inference
        results = self.model(img, verbose=False)

        return results

    def crop_and_run(self, image: PILImage):
        width, height = image.size
        crop_x = int(width * B_PERCENT)
        crop_y = int(height * B_PERCENT)
        crop_width = int(width - (2 * B_PERCENT * width))
        crop_height = int(height - (2 * B_PERCENT * height))
        checkerboard_img = image.crop(
            (crop_x, crop_y, crop_x + crop_width, crop_y + crop_height)
        )

        checkerboard_width, checkerboard_height = checkerboard_img.size

        # Calculate the size of each cell
        cell_width = checkerboard_width / 8
        cell_height = checkerboard_height / 8

        output_matrix = []
        images = []

        # Iterate over each row and column
        for row in range(8):
            for col in range(8):
                # Calculate the left, upper, right, and lower coordinates for each cell
                left = round(col * cell_width)
                upper = round(row * cell_height)
                right = round(left + cell_width)
                lower = round(upper + cell_height)
                crop_box = (left, upper, right, lower)

                # Crop the cell from the image
                cell_image = checkerboard_img.crop(crop_box)

                cell_image = cell_image.resize(RESIZE_TO)

                images.append(cell_image)

        results = self.predict(images)

        draw_img = image.copy().resize(
            (image.size[0] * DEBUG_MUL, image.size[1] * DEBUG_MUL)
        )

        offset_x = round(B_PERCENT * draw_img.size[0])
        offset_y = round(B_PERCENT * draw_img.size[1])

        cell_width *= DEBUG_MUL
        cell_height *= DEBUG_MUL
        # Iterate over each row and column
        index = 0
        for row in range(8):
            row_classifications = [].copy()
            for col in range(8):
                left = round(col * cell_width) + offset_x
                upper = round(row * cell_height) + offset_y
                right = round(left + cell_width)
                lower = round(upper + cell_height)

                classifications = results[index]
                index += 1

                names = classifications.names
                probs = classifications.probs.data

                actual_names = []
                for key in range(13):
                    actual_names.append(names[key])

                confidence_scores = {}
                for label, confidence in zip(actual_names, probs):
                    confidence_scores[label] = float(confidence)

                row_classifications.append(confidence_scores)

                label = sorted(confidence_scores, key=confidence_scores.get)[-1]

                draw_img = draw_bb(draw_img, left, upper, right, lower, label)
            output_matrix.append(row_classifications)

        return draw_img, output_matrix
