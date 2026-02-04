import csv
import math
import sys
import os

def main(filepath):
    filename = os.path.splitext(os.path.basename(filepath))[0]
    save_dir = os.path.dirname(filepath)

    # Output KML files
    output_kml_points = os.path.join(save_dir, filename + "_points.kml")
    output_kml_lines = os.path.join(save_dir, filename + "_lines.kml")

    def rad_to_deg(r):
        return math.degrees(float(r))
    
    # Draw a short line (arrow) at the coordinates extending in the heading direction
    def heading_arrow(lat, lon, heading_deg, length_m=2):
        R = 6371000  # Earth radius (m)
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        brng = math.radians(heading_deg)
        lat2 = math.asin(math.sin(lat1)*math.cos(length_m/R) +
                        math.cos(lat1)*math.sin(length_m/R)*math.cos(brng))
        lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(length_m/R)*math.cos(lat1),
                                math.cos(length_m/R)-math.sin(lat1)*math.sin(lat2))
        return math.degrees(lat2), math.degrees(lon2)

    points = []

    # --- Read CSV ---
    with open(filepath, newline='') as f:
        reader = csv.DictReader(f)
        reader.fieldnames = [h.strip() for h in reader.fieldnames]
        for row in reader:
            lat = float(row['latitude'])
            lon = float(row['longitude'])
            gps_heading_deg = rad_to_deg(row['current GPS heading'])
            desired_heading_deg = rad_to_deg(row['desired GPS heading'])
            count = row['Count']
            points.append({
                'lat': lat,
                'lon': lon,
                'gps_heading_deg': gps_heading_deg,
                'desired_heading_deg': desired_heading_deg,
                'count': count
            })

    # Create the list of points for the points KML file 
    with open(output_kml_points, "w") as f:
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
""")
        for p in points:
            f.write(f"""
    <Placemark>
        <name>{p['count']}</name>
        <description><![CDATA[
        <b>GPS Heading:</b> {p['gps_heading_deg']:.2f}°<br/>
        <b>Desired GPS Heading:</b> {p['desired_heading_deg']:.2f}°
        ]]></description>
        <Point>
            <coordinates>{p['lon']},{p['lat']},0</coordinates>
        </Point>
    </Placemark>
""")
        f.write("</Document>\n</kml>")

    # Create a list of lines (representing headings) for the lines KML file
    with open(output_kml_lines, "w") as f:
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
""")
        for p in points:
            # Current GPS GPS heading arrow (green)
            if p['gps_heading_deg'] != 0:
                arrow_end_lat, arrow_end_lon = heading_arrow(p['lat'], p['lon'], p['gps_heading_deg'])
                f.write(f"""
    <Placemark>
        <Style>
            <LineStyle>
                <color>008000</color>  <!-- Green -->
                <width>2</width>
            </LineStyle>
        </Style>
        <LineString>
            <coordinates>
                {p['lon']},{p['lat']},0
                {arrow_end_lon},{arrow_end_lat},0
            </coordinates>
        </LineString>
    </Placemark>
""")
            # Desired heading arrow (gold)
            if p['desired_heading_deg'] != 0:
                arrow_end_lat, arrow_end_lon = heading_arrow(p['lat'], p['lon'], p['desired_heading_deg'])
                f.write(f"""
    <Placemark>
        <Style>
            <LineStyle>
                <color>ff37afd4</color>  <!-- Gold -->
                <width>2</width>
            </LineStyle>
        </Style>
        <LineString>
            <coordinates>
                {p['lon']},{p['lat']},0
                {arrow_end_lon},{arrow_end_lat},0
            </coordinates>
        </LineString>
    </Placemark>
""")
        f.write("</Document>\n</kml>")

    print(f"Points KML saved: {output_kml_points}")
    print(f"Lines KML saved: {output_kml_lines}")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("ERROR, invalid number of arguments. \nUsage: python plot_pid_logs.py <path_to_csv>")
        sys.exit(1)
    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"Error: File '{csv_path}' does not exist.")
        sys.exit(1)
    main(csv_path)
