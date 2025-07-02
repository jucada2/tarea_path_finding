//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include <iostream>
#include <queue>

#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <set>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    AStar
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist > other.dist;
        }
    };

    void dijkstra(Graph &graph) {
        // --- NUEVO CÓDIGO DE DEPURACIÓN ---
        std::cout << "--- Iniciando Dijkstra ---" << std::endl;
        std::cout << "ID Origen (src): " << src->id << " | ID Destino (dest): " << dest->id << std::endl;
        // ---------------------------------
        std::unordered_map<Node*, double> dists;
        std::unordered_map<Node *, Node *> parent;

        // Usamos una priority_queue de C++ que es más eficiente para Dijkstra.
        // El `>` sobrecargado en `Entry` la convierte en una cola de mínima prioridad.
        std::priority_queue<Entry> pq;

        // Conjunto para guardar los nodos cuyo camino más corto ya es definitivo
        std::set<Node*> finalized_nodes;
        int nodes_processed = 0;
        // 1. Inicialización
        for (auto const& [id, node] : graph.nodes) {
            dists[node] = std::numeric_limits<double>::max();
        }
        dists[src] = 0.0;
        pq.push({src, 0.0});

        // 2. Bucle principal del algoritmo
        while (!pq.empty()) {
            Node* u = pq.top().node;
            pq.pop();

            // --- AÑADE ESTAS LÍNEAS ---
            nodes_processed++;
            // Imprime el progreso cada 50,000 nodos procesados
            if (nodes_processed % 50000 == 0) {
                render(graph);
                std::cout << "Nodos procesados: " << nodes_processed << "..." << std::endl;
            }
            // ------------------------

            if (finalized_nodes.count(u)) {
                continue;
            }
            finalized_nodes.insert(u);
            // Para visualización: pintamos el nodo actual
            if (u != src && u != dest) {
                u->color = sf::Color::Blue;
            }
            // Si llegamos al destino, podemos detenernos
            if (u == dest) break;

            // 3. Explorar los vecinos ('v') del nodo actual ('u')
            for (Edge* edge : u->edges) {

                if (edge == nullptr) {
                    continue;
                }
                // Determinar el vecino correcto (dependiendo de la dirección de la arista)
                Node* v = (edge->src == u) ? edge->dest : edge->src;
                if (v == nullptr) {
                    continue;
                }

                // Ignorar aristas unidireccionales en la dirección incorrecta
                if (edge->one_way && edge->dest != v) continue;

                double weight = edge->length;

                if (dists[u] + weight < dists[v]) {
                    visited_edges.emplace_back(u->coord, v->coord, sf::Color(255, 255, 0, 100), 1.5f);
                    dists[v] = dists[u] + weight;
                    parent[v] = u;
                    pq.push({v, dists[v]});
                }
            }
        }
        std::cout << "Búsqueda terminada. Total de nodos procesados: " << nodes_processed << std::endl;
        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        // TODO: Add your code here

        set_final_path(parent);
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render(Graph& graph) {
        // Procesamos eventos para que la ventana no se congele
        sf::Event event;
        while(window_manager->poll_event(event)) {
            if(event.type == sf::Event::Closed) window_manager->close();
        }

        window_manager->clear();          // Limpia el fotograma anterior
        graph.draw();                     // Dibuja el estado base del grafo
        draw(true);                       // Dibuja las aristas visitadas y los nodos src/dest
        window_manager->display();        // Muestra el nuevo fotograma
        sf::sleep(sf::milliseconds(10));
        // TODO: Add your code here
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        // Empezamos desde el nodo de destino
        Node* current = dest;

        // Recorremos el mapa de padres hacia atrás hasta que no haya más
        // (o hasta que lleguemos al inicio, que no tiene padre)
        while (parent.count(current)) {
            Node* p = parent[current];

            // Creamos una línea verde y más gruesa para el camino final
            path.emplace_back(p->coord, current->coord, sf::Color::Green, 3.0f);

            // Nos movemos al nodo padre para continuar el trazado
            current = p;
        }

        // El camino se construyó desde el final hacia el inicio,
        // así que lo invertimos para tenerlo en el orden correcto.
        std::reverse(path.begin(), path.end());
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }

        // Limpiamos solo los caminos, sin borrar src y dest
        path.clear();
        visited_edges.clear();

        // Aseguramos que src y dest mantengan su color distintivo
        src->color = sf::Color::Green;
        dest->color = sf::Color::Cyan;

        switch (algorithm) {
            case Dijkstra:
                // Llama a la función interna dijkstra
                dijkstra(graph);
                break;
            case AStar:
                // Llama a la función interna a_star
                a_star(graph);
                break;
            case None:
                // Si el algoritmo es 'None' o cualquier otro caso, no hace nada
            default:
                break;
        }
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
