# Bit    Mask      Description
#  0     0x01      GPS L1 used in Solution
#  1     0x02      GPS L2 used in Solution
#  2     0x04      GPS L5 used in Solution
#  3     0x08      BD2 B3 used in Solution
#  4     0x10      GLONASS L1 used in Solution
#  5     0x20      GLONASS L2 used in Solution
#  6     0x40      BD2 B1 used in Solution
#  6     0x80      BD2 B2 used in Solution
uint32 original_mask
bool gps_L1_used_in_solution
bool gps_L2_used_in_solution
bool gps_L5_used_in_solution
bool glonass_L1_used_in_solution
bool glonass_L2_used_in_solution
bool BDS_B1_used_in_solution
bool BDS_B2_used_in_solution
bool BDS_B3_used_in_solution