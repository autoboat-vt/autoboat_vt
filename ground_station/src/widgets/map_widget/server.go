package main

import (
	"log"
	"net/http"

	"github.com/gorilla/mux"
	jsoniter "github.com/json-iterator/go"
	"github.com/rs/cors"
)

var (
	json      = jsoniter.ConfigCompatibleWithStandardLibrary
	waypoints = make([][]float64, 0)
)

// Handler for POST /waypoints
func updateWaypointsHandler(w http.ResponseWriter, r *http.Request) {

	// clear the existing waypoints before adding new ones
	waypoints = waypoints[:0]

	var reqBody struct {
		Waypoints [][]float64 `json:"waypoints"`
	}

	// decode the request body and store it into reqBody
	var err = json.NewDecoder(r.Body).Decode(&reqBody)

	if err != nil {
		http.Error(w, `{"message": "Invalid request body"}`, http.StatusBadRequest)
		return
	}

	waypoints = reqBody.Waypoints

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
}

// Handler for GET /waypoints
func getWaypointsHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(waypoints)
}

func main() {
	var router = mux.NewRouter()

	router.HandleFunc("/waypoints", updateWaypointsHandler).Methods("POST")
	router.HandleFunc("/waypoints", getWaypointsHandler).Methods("GET")

	var handler = cors.Default().Handler(router)

	var port = ":3001"
	log.Printf("[Go] Local waypoints server running at http://localhost%s\n", port)

	var err = http.ListenAndServe(port, handler)

	if err != nil {
		log.Fatalf("Could not start server: %s\n", err)
	}
}
