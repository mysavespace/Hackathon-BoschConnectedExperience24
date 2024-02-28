import cv2
import numpy as np

maxint = 2147483647

def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1*(3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])

def arragne_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
    left_fit_average = np.average(left_fit, axis=0) if left_fit else None
    right_fit_average = np.average(right_fit, axis=0) if right_fit else None
    left_line = make_coordinates(image, left_fit_average) if left_fit_average is not None else None
    right_line = make_coordinates(image, right_fit_average) if right_fit_average is not None else None

    return np.array([left_line, right_line]) if left_line is not None and right_line is not None else None

def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    canny = cv2.Canny(blur, 150, 200)
    return canny

def display_lins(image, lines):
    line_img = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            if line is not None:
                x1, y1, x2, y2 = line.reshape(4)
                if all(isinstance(i, (int, float, np.number)) for i in [x1, y1, x2, y2]):
                    if((x1 < 0 or y1 < 0 or x2 < 0 or y2 < 0) or (x1 > maxint or y1 > maxint or x2 > maxint or y2 > maxint)):
                        # print("Not found anything")
                        return 1

                    # print(x1, y1, x2, y2)
                    cv2.line(line_img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 10)
    return 0


# right left line works
def region_of_interest(image):
    height = image.shape[0]
    width = image.shape[1]
    polygons = np.array([[
        (0, height),
        (width / 2, 200),
        (1800, height),]], np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)

    # Create an inner mask with a 100px border
    inner_polygons = np.array([[
        (600, height - 100),
        (width / 2, 300),
        (width - 600, height - 100),]], np.int32)
    inner_mask = np.ones_like(image) * 255
    cv2.fillPoly(inner_mask, inner_polygons, 0)
    final_image = cv2.bitwise_and(masked_image, inner_mask)

    return final_image

# parking lines
# def region_of_intrest_2(image):
#     height = image.shape[0]
#     width = image.shape[1]

#     polygons = np.array(
#         [
#             [
#                 (width / 2 + 500, 800),  # Bottom center
#                 (width / 2 + 300, 300),  # Top center
#                 (width, 500),  # Top right
#                 (width, 1000),  # Bottom right
#             ]
#         ], np.int32)

#     mask = np.zeros_like(image)
#     cv2.fillPoly(mask, polygons, 255)
#     masked_image = cv2.bitwise_and(image, mask)
#     return masked_image

def doImage_old(src):
    image = cv2.imread(src)
    lane_image = np.copy(image)
    canny_image = canny(lane_image)  # renamed variable
    cropped_image = region_of_intrest(canny_image)  # use the renamed variable
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=20, maxLineGap=5)
    arranged_lines = arragne_slope_intercept(lane_image, lines)
    line_image = display_lins(lane_image, arranged_lines)
    return line_image
    # combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
    # cv2.imshow("result",combo_image)
    # cv2.waitKey(0)

import base64

def readb64(uri):
   encoded_data = uri.split(',')[1]
   nparr = np.fromstring(base64.b64decode(encoded_data), np.uint8)
   img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
   return img

def doImage(image):
    lane_image = np.copy(image)
    canny_image = canny(lane_image)  # renamed variable
    cropped_image = region_of_interest(canny_image)  # use the renamed variable
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=20, maxLineGap=5)
    arranged_lines = arragne_slope_intercept(lane_image, lines)
    line_image = display_lins(lane_image, arranged_lines)
    return line_image


def doVideo(src):
    cap = cv2.VideoCapture(src)
    failed = 0
    while(cap.isOpened()):
        _, frame = cap.read()
        canny_image = canny(frame)
        cropped_image = region_of_intrest(canny_image)
        lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=20, maxLineGap=5)
        arranged_lines = arragne_slope_intercept(frame, lines)
        line_image = display_lins(frame, arranged_lines)
        if(line_image is False):
            failed += 1
            if(failed >= 5):
                break
        else:
            failed = 0
        combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        cv2.imshow("result",combo_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# doVideo("test2.mp4")
# doImage("parking.png")