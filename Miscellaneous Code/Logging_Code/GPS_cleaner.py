import csv
import sys
import os

def remove_duplicate_coordinates(input_csv, output_csv):
    coord_dict = {}  # key: (lat, lon), value: row

    with open(input_csv, newline='') as f:
        reader = csv.DictReader(f)
        reader.fieldnames = [h.strip() for h in reader.fieldnames]
        fieldnames = reader.fieldnames.copy()

        for row in reader:
            lat = float(row['latitude'])
            lon = float(row['longitude'])
            gps_heading = float(row['current GPS heading'])
            key = (lat, lon)

            # If the coordinate is not yet in dict, just add it
            if key not in coord_dict:
                coord_dict[key] = row
            else:
                # If this row has non-zero GPS heading, replace the existing one
                if gps_heading != 0:
                    coord_dict[key] = row
                # Otherwise, keep the existing one (could be zero or previously replaced)

    # Sort the rows by chronological order, this is important so it renders properly in My Maps
    filtered_rows = sorted(coord_dict.values(), key=lambda r: float(r['Timestamp']))

    # Add a new 'Count' column at the start
    new_fieldnames = ['Count'] + fieldnames

    with open(output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=new_fieldnames)
        writer.writeheader()
        for i, row in enumerate(filtered_rows, start=1):
            row_with_line = {'Count': i}
            row_with_line.update(row)
            writer.writerow(row_with_line)

    print(f"Filtered CSV saved to {output_csv}")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python remove_duplicates.py <input_csv>")
        sys.exit(1)

    input_csv = sys.argv[1]
    if not os.path.exists(input_csv):
        print(f"Error: File '{input_csv}' does not exist.")
        sys.exit(1)

    output_csv = os.path.splitext(input_csv)[0] + "_filtered.csv"
    remove_duplicate_coordinates(input_csv, output_csv)
