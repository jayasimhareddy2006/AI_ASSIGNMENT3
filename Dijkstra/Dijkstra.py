import csv
import heapq

def load_graph_from_csv(file_path):
    graph = {}

    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # skip header

        for row in reader:
            src, dest, dist = row[0], row[1], int(row[2])

            if src not in graph:
                graph[src] = {}
            if dest not in graph:
                graph[dest] = {}

            # Undirected graph (roads go both ways)
            graph[src][dest] = dist
            graph[dest][src] = dist

    return graph


def dijkstra(graph, start):
    distances = {city: float('inf') for city in graph}
    distances[start] = 0

    pq = [(0, start)]

    while pq:
        current_distance, current_city = heapq.heappop(pq)

        if current_distance > distances[current_city]:
            continue

        for neighbor, weight in graph[current_city].items():
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))

    return distances


def main():
    graph = load_graph_from_csv("cities.csv")

    print("Available Cities:")
    for city in graph.keys():
        print(city)

    start = input("\nEnter starting city: ")

    if start not in graph:
        print("City not found!")
        return

    distances = dijkstra(graph, start)

    print(f"\nShortest distances from {start}:\n")
    for city, dist in distances.items():
        print(f"{city} → {dist} km")


if __name__ == "__main__":
    main()
