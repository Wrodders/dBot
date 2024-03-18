import argparse
import math
from PIL import Image, ImageDraw

class ShapeGenerator:
    def __init__(self, width, height, shape_type, orientation):
        self.width = width
        self.height = height
        self.shape_type = shape_type
        self.orientation = orientation
        self.image = Image.new("1", (width, height), color="black")
        self.draw = ImageDraw.Draw(self.image)

    def generate_shape_vertices(self, side_length, center_x, center_y, height=None):
        if self.shape_type == "hexagon":
            return self.generate_hexagon_vertices(side_length, center_x, center_y)
        elif self.shape_type == "circle":
            return self.generate_circle_vertices(side_length, center_x, center_y)
        elif self.shape_type == "octagon":
            return self.generate_octagon_vertices(side_length, center_x, center_y)
        elif self.shape_type == "rectangle":
            return self.generate_rectangle_vertices(side_length, center_x, center_y, height)
        else:
            raise ValueError("Unsupported shape type")

    def generate_hexagon_vertices(self, side_length, center_x, center_y):
        vertical_distance = side_length * math.sqrt(3) / 2
        vertices = [
            (center_x - 1.5 * side_length, center_y),
            (center_x - 0.5 * side_length, center_y + vertical_distance),
            (center_x + 0.5 * side_length, center_y + vertical_distance),
            (center_x + 1.5 * side_length, center_y),
            (center_x + 0.5 * side_length, center_y - vertical_distance),
            (center_x - 0.5 * side_length, center_y - vertical_distance)
        ]
        return vertices

    def generate_circle_vertices(self, radius, center_x, center_y, num_points=30):
        angle_increment = 2 * math.pi / num_points
        vertices = [
            (
                center_x + radius * math.cos(angle),
                center_y + radius * math.sin(angle)
            )
            for angle in (angle_increment * i for i in range(num_points))
        ]
        return vertices

    def generate_octagon_vertices(self, side_length, center_x, center_y):
        angle = 2 * math.pi / 8
        vertices = [
            (center_x + side_length * math.cos(angle * i), center_y + side_length * math.sin(angle * i))
            for i in range(8)
        ]
        return vertices

    def generate_rectangle_vertices(self, side_length, center_x, center_y, height=None):
        half_width = side_length // 2
        half_height = height or side_length // 2
        vertices = [
            (center_x - half_width, center_y - half_height),
            (center_x + half_width, center_y - half_height),
            (center_x + half_width, center_y + half_height),
            (center_x - half_width, center_y + half_height)
        ]
        return vertices

    def draw_shape(self, *args, **kwargs):
        shape_vertices = self.generate_shape_vertices(*args, **kwargs)
        self.draw.polygon(shape_vertices, fill="white", outline="white")

    def save_image(self, output_path):
        self.image.save(output_path)
        print(f"Image saved to {output_path}.")

def generate_shapes(args):
    width = args.width
    height = args.height
    shape_type = args.shape_type
    side_length = args.side_length
    spacing = args.spacing
    rectangle_height = args.rectangle_height
    orientation = args.orientation
    output = args.output

    shape_generator = ShapeGenerator(width, height, shape_type, orientation)

    center_x = width // 2
    center_y = height // 2

    if orientation == "vertical":
        spacing_x, spacing_y = 0, spacing
    elif orientation == "horizontal":
        spacing_x, spacing_y = spacing, 0

    shape_generator.draw_shape(side_length, center_x - spacing_x // 2, center_y - spacing_y // 2, height=rectangle_height)
    shape_generator.draw_shape(side_length, center_x + spacing_x // 2, center_y + spacing_y // 2, height=rectangle_height)

    shape_generator.save_image(output)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate images with shapes.")
    parser.add_argument("width", type=int, help="Width of the image")
    parser.add_argument("height", type=int, help="Height of the image")
    parser.add_argument("shape_type", type=str, help="Type of shape to draw")
    parser.add_argument("side_length", type=int, help="Side length for the shape (or radius for circle)")
    parser.add_argument("spacing", type=int, help="Spacing between shapes (for hexagon, octagon, and rectangle)")
    parser.add_argument("--rectangle_height", type=int, help="Height for the rectangle, defaults to side_length if not provided")
    parser.add_argument("--orientation", type=str, default="horizontal", choices=["horizontal", "vertical"], help="Orientation of shapes: 'horizontal' or 'vertical'")
    parser.add_argument("--output", type=str, default="bmp/output.bmp", help="Output file path")

    args = parser.parse_args()
    generate_shapes(args)
