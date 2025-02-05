from camera import BackupCamera

def main():

    camera = BackupCamera(address='192.168.69.41',
                          username='admin',
                          password='')
    camera.start_video()


if __name__ == '__main__':
    main()
