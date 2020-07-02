#std_msgs/Header header
int32 id
string gender
int32 age_years
string age_type
float64 emotion_neutral
float64 emotion_angry
float64 emotion_happy
float64 emotion_surprise
#normalized screen coordinates of a face rectangle in the image (x, y, w, h)
float64[] rectangle
#normalized coordinates of the center of a person’s left eye (x,y)
float64[] left_eye
#normalized coordinates of the center of a person’s right eye (x,y)
float64[] right_eye
#face orientation angles in degrees (yaw, pitch, roll)
float64[] angles
