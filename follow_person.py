from go_to_person_node import GoToPersonNode
import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = GoToPersonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
