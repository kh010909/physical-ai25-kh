import cv2
import numpy as np

points = []

class Projection(object):

    def __init__(self, image_path, points):
        """
            :param points: Selected pixels on top view(BEV) image
        """

        if type(image_path) != str:
            self.image = image_path
        else:
            self.image = cv2.imread(image_path)
        self.height, self.width, self.channels = self.image.shape

    def top_to_front(self, theta=0, phi=0, gamma=0, dx=0, dy=0, dz=0, fov=90):
        """
            Project the top view pixels to the front view pixels.
            :return: New pixels on perspective(front) view image
        """
        # Camera Intrinsic Parameters
        img_w, img_h = 512, 512
        c_x, c_y = img_w / 2, img_h / 2
        f_x = f_y = c_x / np.tan(np.deg2rad(fov / 2))
        intrinsic_matrix = np.array([[f_x, 0, c_x],
                                     [0, f_y, c_y],
                                     [0, 0, 1]])
        intrinsic_inv = np.linalg.inv(intrinsic_matrix)

        # Front Camera (Cam1) Pose
        rotation_front = np.eye(3)
        translation_front = np.array([[0], [1], [0]])

        # BEV Camera (Cam2) Pose
        pitch_rad = -np.pi / 2
        rotation_bev = np.array([[1, 0, 0],
                                 [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
                                 [0, np.sin(pitch_rad), np.cos(pitch_rad)]])
        translation_bev = np.array([[0], [2.5], [0]])

        # World-to-Front-Camera Transformation
        transform_world_front = np.vstack((np.hstack((rotation_front, translation_front)), 
                                          [0, 0, 0, 1]))
        transform_front_world = np.linalg.inv(transform_world_front)
        
        projected_pixels = []
        for bev_pixel in points:
            # Unproject BEV pixel to 3D ray
            pixel_homog = np.array([bev_pixel[0], bev_pixel[1], 1.0])
            camera_ray = intrinsic_inv @ pixel_homog

            # Transform ray to world coordinates
            world_ray = rotation_bev @ camera_ray
            
            # Intersect with ground plane (Y=0)
            scale = -translation_bev[1] / world_ray[1]
            world_point = translation_bev.flatten() + scale * world_ray
            world_point_homog = np.append(world_point, 1)

            # Transform to front camera coordinates
            front_cam_point_homog = transform_front_world @ world_point_homog
            
            # Project onto front camera image plane
            front_cam_point = front_cam_point_homog[:3] / front_cam_point_homog[2]
            front_pixel_homog = intrinsic_matrix @ front_cam_point
            
            u_coord, v_coord = int(front_pixel_homog[0]), int(front_pixel_homog[1])

            if 0 <= u_coord < img_w and 0 <= v_coord < img_h:
                projected_pixels.append([u_coord, v_coord])
                
        return projected_pixels

    def show_image(self, new_pixels, img_name='projection.png', color=(0, 0, 255), alpha=0.4):
        """
            Show the projection result and fill the selected area on perspective(front) view image.
        """

        new_image = cv2.fillPoly(
            self.image.copy(), [np.array(new_pixels)], color)
        new_image = cv2.addWeighted(
            new_image, alpha, self.image, (1 - alpha), 0)

        cv2.imshow(
            f'Top to front view projection {img_name}', new_image)
        cv2.imwrite(img_name, new_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return new_image


def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:

        print(x, ' ', y)
        points.append([x, y])
        font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(img, str(x) + ',' + str(y), (x+5, y+5), font, 0.5, (0, 0, 255), 1)
        cv2.circle(img, (x, y), 3, (0, 0, 255), -1)
        cv2.imshow('image', img)

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:

        print(x, ' ', y)
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        # cv2.putText(img, str(b) + ',' + str(g) + ',' + str(r), (x, y), font, 1, (255, 255, 0), 2)
        cv2.imshow('image', img)


if __name__ == "__main__":

    pitch_ang = -90

    front_rgb = "bev_data/front1.png"
    top_rgb = "bev_data/bev1.png"

    # click the pixels on window
    img = cv2.imread(top_rgb, 1)
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    projection = Projection(front_rgb, points)
    new_pixels = projection.top_to_front(theta=pitch_ang)
    projection.show_image(new_pixels)
