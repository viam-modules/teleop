// Command module is the viam:teleop module entrypoint.
package main

import (
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"

	"teleop/services/clutch"
	"teleop/services/leaderfollower"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: generic.API, Model: clutch.Model},
		resource.APIModel{API: generic.API, Model: leaderfollower.Model},
	)
}
