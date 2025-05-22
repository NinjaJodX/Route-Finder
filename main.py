import streamlit as st
import osmnx as ox
import networkx as nx
import folium
from streamlit_folium import folium_static
from geopy.geocoders import Nominatim
import time


# Function to fetch road network data
def get_road_network(city):
    try:
        G = ox.graph_from_place(city, network_type="drive")
        return G
    except Exception:
        return None


# Function to convert address to coordinates
def get_coordinates(location):
    geolocator = Nominatim(user_agent="route_finder")
    loc = geolocator.geocode(location)
    return (loc.latitude, loc.longitude) if loc else None


# Function to get the nearest graph node
def get_nearest_node(G, location):
    coordinates = get_coordinates(location)
    if coordinates:
        lat, lon = coordinates
        return ox.distance.nearest_nodes(G, lon, lat)
    return None


# Dijkstra's Algorithm for shortest path
def shortest_path_dijkstra(G, start, end):
    return nx.shortest_path(G, source=start, target=end, weight="length")


# A* Algorithm for shortest path
def shortest_path_astar(G, start, end):
    return nx.astar_path(G, source=start, target=end, weight="length")


# Function to calculate total route distance
def calculate_distance(G, path):
    total_distance = 0
    for i in range(len(path) - 1):
        edge_data = G.get_edge_data(path[i], path[i + 1])
        if edge_data is not None:
            total_distance += edge_data[0]["length"]
    return total_distance / 1000  # Convert meters to kilometers


# Function to plot the route on a map
def plot_route(G, path, start, end):
    start_coords = get_coordinates(start)
    route_map = folium.Map(location=start_coords, zoom_start=14)
    route_coordinates = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in path]
    folium.PolyLine(route_coordinates, color="blue", weight=5, opacity=0.7).add_to(route_map)
    folium.Marker(route_coordinates[0], popup="Start", icon=folium.Icon(color="green")).add_to(route_map)
    folium.Marker(route_coordinates[-1], popup="End", icon=folium.Icon(color="red")).add_to(route_map)
    return route_map


# Streamlit UI
def main():
    st.set_page_config(page_title="🚗 Route Finder", layout="wide")

    st.title("🚗 Route Finder Web App")

    with st.sidebar:
        st.subheader("🌍 Enter Location Details")
        city = st.text_input("City (e.g., Varanasi, UP):")
        start_location = st.text_input("Start Location:")
        end_location = st.text_input("End Location:")
        find_routes = st.button("Find Routes")

    if find_routes and city:
        with st.spinner("Fetching road network..."):
            G = get_road_network(city)

        if G:
            with st.spinner("Locating start and end points..."):
                start_node = get_nearest_node(G, start_location)
                end_node = get_nearest_node(G, end_location)

            if start_node and end_node:
                with st.spinner("Calculating shortest routes..."):
                    # Measure time for Dijkstra's algorithm
                    start_time = time.time()
                    path_dijkstra = shortest_path_dijkstra(G, start_node, end_node)
                    dijkstra_time = time.time() - start_time
                    dijkstra_distance = calculate_distance(G, path_dijkstra)

                    # Measure time for A* algorithm
                    start_time = time.time()
                    path_astar = shortest_path_astar(G, start_node, end_node)
                    astar_time = time.time() - start_time
                    astar_distance = calculate_distance(G, path_astar)

                st.success("✅ Route Calculation Complete!")

                # Displaying maps in a column-wise format
                st.subheader("📍 Dijkstra's Algorithm")
                st.info(f"Distance: {dijkstra_distance:.2f} km\nTime Taken: {dijkstra_time:.4f} sec")
                folium_static(plot_route(G, path_dijkstra, start_location, end_location))

                st.write("""
                    ---
                """)  # Adds space between maps

                st.subheader("📍 A* Algorithm")
                st.info(f"Distance: {astar_distance:.2f} km\nTime Taken: {astar_time:.4f} sec")
                folium_static(plot_route(G, path_astar, start_location, end_location))
                if dijkstra_time > astar_time:
                    st.subheader("Therefore, Dijkstar Algorithm takes more time then Astar Algorithm.")
                else:
                    st.subheader("Therefore, Astar Algorithm takes more time then Dijkstar Algorithm.")

            else:
                st.error("❌ Invalid start or end location. Please try again.")
        else:
            st.error("❌ Could not fetch road network for the given city.")

if __name__ == "__main__":
    main()