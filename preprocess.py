import glob, os

#Current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

print(current_dir)

current_dir = '/home/nicholasward2/darknet/build/darknet/x64/data/obj'

# Per of images to be used for the test set

percentage_test = 10

#Create and/or truncate train.txt and test.txt

file_train = open('/home/nicholasward2/darknet/build/darknet/x64/data/train.txt', 'w')
file_test = open('/home/nicholasward2/darknet/build/darknet/x64/data/test.txt', 'w')


# Add images to train.txt and test.txt

counter = 1
index_text = round(100 / percentage_test)

for path_and_filename in glob.iglob(os.path.join(current_dir, '*.jpg')):
    title, ext = os.path.splitext(os.path.basename(path_and_filename))
    
    if counter == index_test:
        counter = 1
        file_test.write('/home/nicholasward2/darknet/build/darknet/x64/data/obj' + '/' + title + '.jpg' + '\n')
        
    else: 
        file_train.write('/home/nicholasward2/darknet/build/darknet/x64/data/obj' + '/' + title + '.jpg' + '\n')
        counter = counter + 1
